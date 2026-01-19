<script lang="ts" module>
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Start/Stop';
	export const description =
		'Start/Stop autonomy by calling ROS2 services + show waypoint table.';
	export const group: WidgetGroupType = 'ROS';
	export const isRosDependent = true;

	// YAMLs are served by the Svelte app (HTTP paths)
	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			waypoints: {
				yamlFile: {
					type: 'select',
					value: '/waypoints_simulation.yaml',
					options: [
						{ label: 'waypoints.yaml', value: '/waypoints.yaml' },
						{ label: 'waypoints_simulation.yaml', value: '/waypoints_simulation.yaml' },
					]
				}
			}
		}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte';
	import * as ROSLIB from 'roslib';
	import { Button } from '$lib/components/ui/button/index.js';
	import { onMount } from 'svelte';

	// ---------------- ROS (Services) ----------------
	let runSrv = $state<ROSLIB.Service | null>(null);
	let cancelSrv = $state<ROSLIB.Service | null>(null);

	// ---------------- UI state ----------------
	let isStarted = $state(false);

	// Service status / busy
	let svcStatus = $state<string>('ROS disconnected');
	let svcBusy = $state(false);

	// Navigation info subscription (internal, not exposed as state)
	let navInfoSub: ROSLIB.Topic | null = null;

	// ---------------- Hold-to-stop ----------------
	const HOLD_MS = 3000;
	let isHoldingStop = $state(false);
	let holdProgress = $state(0);
	let holdTimer: number | null = null;
	let progressTimer: number | null = null;
	let holdStartTs = 0;

	const clearHold = () => {
		isHoldingStop = false;
		holdProgress = 0;
		if (holdTimer) clearTimeout(holdTimer);
		if (progressTimer) clearInterval(progressTimer);
		holdTimer = null;
		progressTimer = null;
	};

	// ---------------- Waypoints ----------------
	type Waypoint = { name: string; x: number; y: number; yaw?: number };

	let waypoints = $state<Waypoint[]>([]);
	let yamlError = $state<string | null>(null);
	let isLoadingYaml = $state(false);

	// Selected YAML HTTP path (used for preview table fetch)
	const getSelectedYaml = () => settings.groups.waypoints.yamlFile.value || '/waypoints.yaml';

	const getSelectedYamlBasename = () => {
		const p = getSelectedYaml();
		const last = p.split('/').pop();
		return last ?? 'waypoints.yaml';
	};

	const loadWaypointsYaml = async () => {
		try {
			isLoadingYaml = true;
			yamlError = null;

			const url = getSelectedYaml(); // HTTP path
			const res = await fetch(url, { cache: 'no-store' });
			if (!res.ok) throw new Error(`Failed to fetch ${url}`);

			const lines = (await res.text())
				.split('\n')
				.map((l) => l.trim())
				.filter((l) => l && !l.startsWith('#') && l !== 'waypoints:');

			const parsed: any[] = [];
			let cur: any = null;

			for (const l of lines) {
				if (l.startsWith('- ')) {
					if (cur) parsed.push(cur);
					cur = {};
					const rest = l.slice(2);
					if (rest.includes(':')) {
						const [k, ...v] = rest.split(':');
						cur[k.trim()] = v.join(':').trim();
					}
				} else if (cur && l.includes(':')) {
					const [k, ...v] = l.split(':');
					cur[k.trim()] = v.join(':').trim();
				}
			}
			if (cur) parsed.push(cur);

			waypoints = parsed
				.map((w, i) => ({
					name: w.name ?? `WP${i + 1}`,
					x: Number(w.x),
					y: Number(w.y),
					yaw: w.yaw !== undefined ? Number(w.yaw) : undefined
				}))
				.filter((w) => Number.isFinite(w.x) && Number.isFinite(w.y));
		} catch (e: any) {
			yamlError = e?.message ?? 'Failed to load YAML';
			waypoints = [];
		} finally {
			isLoadingYaml = false;
		}
	};

	// Auto-reload when dropdown changes
	$effect(() => {
		void settings.groups.waypoints.yamlFile.value;
		loadWaypointsYaml();
	});

	// ---------------- ROS connection (services) ----------------
	$effect(() => {
		const ros = getRosConnection();
		if (!ros) {
		runSrv = null;
		cancelSrv = null;
		svcStatus = 'ROS disconnected';
		svcBusy = false;			isStarted = false;
			clearHold();
			return;
		}

		runSrv = new ROSLIB.Service({
			ros,
			name: '/autonomy/run_waypoints',
			serviceType: 'perseus_interfaces/srv/ToggleWaypoints'
		});

		cancelSrv = new ROSLIB.Service({
			ros,
			name: '/autonomy/cancel_waypoints',
			serviceType: 'perseus_interfaces/srv/ToggleWaypoints'
		});

		svcStatus = 'ROS connected';
	});

	const sendStart = () => {
		if (!runSrv) {
			svcStatus = 'Run service not ready';
			return;
		}

		svcBusy = true;
		svcStatus = 'Sending run request...';

		const yaml_path = getSelectedYamlBasename();

		runSrv.callService(
			new ROSLIB.ServiceRequest({ yaml_path }),
			(resp: any) => {
				svcBusy = false;
				svcStatus = resp?.message ?? 'Run response received';
				if (resp?.success) isStarted = true;
			},
			(err: any) => {
				svcBusy = false;
				svcStatus = `Run failed: ${err?.toString?.() ?? err}`;
			}
		);
	};

	const startHoldStop = () => {
		if (!isStarted || isHoldingStop || svcBusy) return;

		isHoldingStop = true;
		holdStartTs = Date.now();

		holdTimer = window.setTimeout(() => {
			if (!cancelSrv) {
				svcStatus = 'Cancel service not ready';
				isStarted = false;
				clearHold();
				return;
			}

			svcBusy = true;
			svcStatus = 'Cancelling...';

			cancelSrv.callService(
				new ROSLIB.ServiceRequest({}),
				(resp: any) => {
					svcBusy = false;
					svcStatus = resp?.message ?? 'Cancel response received';
					isStarted = false;
					clearHold();
				},
				(err: any) => {
					svcBusy = false;
					svcStatus = `Cancel failed: ${err?.toString?.() ?? err}`;
					clearHold();
				}
			);
		}, HOLD_MS);

		progressTimer = window.setInterval(() => {
			holdProgress = Math.min(1, (Date.now() - holdStartTs) / HOLD_MS);
		}, 16);
	};

	onMount(() => {
		loadWaypointsYaml();

		// Set up persistent subscription to navigation_info
		const setupNavInfoSub = () => {
			const ros = getRosConnection();
			if (!ros) {
				// Try again later if ROS isn't ready yet
				setTimeout(setupNavInfoSub, 1000);
				return;
			}

			navInfoSub = new ROSLIB.Topic({
				ros,
				name: '/autonomy/navigation_info',
				messageType: 'perseus_interfaces/msg/NavigationData'
			});

		navInfoSub.subscribe((msg: any) => {
			// Sync button state with actual navigation state
			const navActive = msg.navigation_active ?? false;
			if (navActive !== isStarted) {
				isStarted = navActive;
				}
			});
		};

		setupNavInfoSub();

		return () => {
			clearHold();
			if (navInfoSub) {
				navInfoSub.unsubscribe();
			}
		};
	});
</script>

<div class="flex h-full w-full flex-col">
	<!-- CONTENT (this is the part that shrinks) -->
	<div class="flex min-h-0 flex-1 flex-col gap-3 overflow-hidden">
		<!-- Waypoints Table -->
		<div class="rounded-xl border bg-background shadow-sm overflow-hidden flex min-h-0 flex-1 flex-col">
			<div class="flex items-center justify-between border-b px-3 py-2 shrink-0">
				<div class="w-full">
					<p class="text-sm font-semibold">Remaining Waypoints</p>

					<div class="mt-0.5 flex flex-wrap items-center gap-2">
						<p class="text-xs opacity-60">{getSelectedYaml()}</p>

						<select
							class="h-7 rounded-md border bg-background px-2 text-xs hover:bg-accent"
							bind:value={settings.groups.waypoints.yamlFile.value}
						>
							{#each settings.groups.waypoints.yamlFile.options as opt}
								<option value={opt.value}>{opt.label}</option>
							{/each}
						</select>

						<button
							class="rounded-md border px-2 py-1 text-xs hover:bg-accent"
							onclick={loadWaypointsYaml}
						>
							Refresh
						</button>

						{#if isLoadingYaml}
							<span class="text-xs animate-pulse opacity-70">Loading…</span>
						{/if}
					</div>
				</div>
			</div>

			{#if yamlError}
				<div class="px-3 py-2">
					<p class="text-xs font-semibold text-red-500">{yamlError}</p>
				</div>
			{:else}
				<!-- This area will shrink + scroll -->
				<div class="min-h-0 flex-1 overflow-auto">
					<table class="w-full text-sm">
						<thead class="sticky top-0 bg-muted/70">
							<tr class="border-b">
								<th class="px-3 py-2 text-left">Name</th>
								<th class="px-3 py-2 text-right">X</th>
								<th class="px-3 py-2 text-right">Y</th>
								<th class="px-3 py-2 text-right">Yaw</th>
							</tr>
						</thead>
						<tbody>
							{#each waypoints as wp, i}
								<tr class={`border-b hover:bg-accent/50 ${i % 2 === 1 ? 'bg-muted/30' : ''}`}>
									<td class="px-3 py-2 font-medium">{wp.name}</td>
									<td class="px-3 py-2 text-right font-mono">{wp.x.toFixed(1)}</td>
									<td class="px-3 py-2 text-right font-mono">{wp.y.toFixed(1)}</td>
									<td class="px-3 py-2 text-right font-mono">{wp.yaw ?? '—'}</td>
								</tr>
							{/each}
						</tbody>
					</table>
				</div>
			{/if}
		</div>
	</div>

	<!-- FOOTER (always pinned to bottom) -->
	<div class="mt-auto flex shrink-0 flex-col gap-3 pt-3">
		<!-- START (fixed height) -->
		<Button
			class={
				(isStarted
					? 'relative w-full font-bold bg-green-700 ring-2 ring-green-300 text-white'
					: 'w-full font-bold bg-green-600 text-white hover:bg-green-700') +
				' h-12 shrink-0'
			}
			disabled={isStarted || !runSrv}
			onclick={sendStart}
		>
			{#if svcBusy && !isStarted}
				SENDING RUN REQUEST…
			{:else if isStarted}
				AUTONOMY IN PROGRESS
			{:else}
				START
			{/if}
		</Button>

		<!-- STOP (fixed height) -->
		<Button
			class="relative w-full font-bold bg-red-600 text-white overflow-hidden h-12 shrink-0"
			disabled={!isStarted || !cancelSrv || svcBusy}
			onpointerdown={(e) => e.button === 0 && startHoldStop()}
			onpointerup={clearHold}
			onpointerleave={clearHold}
			onpointercancel={clearHold}
		>
			<span
				class="absolute inset-y-0 left-0 bg-white/25"
				style={`width:${holdProgress * 100}%`}
			></span>
			<span class="relative z-10">
				{#if isHoldingStop}
					STOPPING AUTONOMY...
				{:else}
					HOLD TO STOP
				{/if}
			</span>
		</Button>

		<!-- STATUS (single line) -->
		<p class="text-xs opacity-70">
			Status: <span class="font-mono">{svcStatus}</span>
		</p>
	</div>
</div>