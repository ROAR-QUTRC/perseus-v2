<script lang="ts" module>
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Start/Stop';
	export const description =
		'Start/Stop autonomy by calling ROS2 services + show waypoint table.';
	export const group: WidgetGroupType = 'ROS';
	export const isRosDependent = true;

	// Add a setting to choose which YAML file to load from /static
	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			waypoints: {
				yamlFile: {
					type: 'select',
					value: '/waypoints.yaml',
					options: [
						{ label: 'waypoints.yaml', value: '/waypoints.yaml' },
						{ label: 'waypoints_simulation.yaml', value: '/waypoints_simulation.yaml' },
						{ label: 'waypoints_test.yaml', value: '/waypoints_test.yaml' },
						{ label: 'waypoints_arc.yaml', value: '/waypoints_arc.yaml' }
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

	// Selected YAML path (from widget settings)
	const getSelectedYaml = () => settings.groups.waypoints.yamlFile.value || '/waypoints.yaml';

	const loadWaypointsYaml = async () => {
		try {
			isLoadingYaml = true;
			yamlError = null;

			const url = getSelectedYaml();
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
		// Touch the reactive value so this reruns on change
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
			svcBusy = false;

			isStarted = false;
			clearHold();
			return;
		}

		runSrv = new ROSLIB.Service({
			ros,
			name: '/autonomy/run_waypoints',
			serviceType: 'perseus_autonomy_interfaces/srv/RunWaypoints'
		});

		cancelSrv = new ROSLIB.Service({
			ros,
			name: '/autonomy/cancel_waypoints',
			serviceType: 'perseus_autonomy_interfaces/srv/CancelWaypoints'
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

		const yaml_path = getSelectedYaml(); // HTTP path (e.g. "/waypoints.yaml")

		runSrv.callService(
			new ROSLIB.ServiceRequest({ yaml_path }),
			(resp: any) => {
				svcBusy = false;
				svcStatus = resp?.message ?? 'Run response received';
				if (resp?.success) {
					isStarted = true;
				}
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
			// After holding long enough: call cancel service
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
		return () => {
			clearHold();
		};
	});
</script>

<div class="flex h-full w-full flex-col gap-3">
	<!-- Waypoints Table -->
	<div class="rounded-xl border bg-background shadow-sm">
		<div class="flex items-center justify-between border-b px-3 py-2">
			<div class="w-full">
				<p class="text-sm font-semibold">Waypoints</p>

				<div class="mt-0.5 flex flex-wrap items-center gap-2">
					<p class="text-xs opacity-60">{getSelectedYaml()}</p>

					<!-- Dropdown to choose YAML file -->
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

				<p class="mt-1 text-xs opacity-70">
					Status: <span class="font-mono">{svcStatus}</span>
				</p>
			</div>
		</div>

		{#if yamlError}
			<div class="px-3 py-2">
				<p class="text-xs font-semibold text-red-500">{yamlError}</p>
			</div>
		{:else}
			<div class="max-h-52 overflow-auto">
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
								<td class="px-3 py-2 text-right font-mono">{wp.x.toFixed(3)}</td>
								<td class="px-3 py-2 text-right font-mono">{wp.y.toFixed(3)}</td>
								<td class="px-3 py-2 text-right font-mono">{wp.yaw ?? '—'}</td>
							</tr>
						{/each}
					</tbody>
				</table>
			</div>
		{/if}
	</div>

	<!-- START -->
	<Button
		class={isStarted
			? 'relative w-full font-bold bg-green-700 ring-2 ring-green-300 text-white'
			: 'w-full font-bold bg-green-600 text-white hover:bg-green-700'}
		disabled={isStarted || !runSrv || svcBusy}
		onclick={sendStart}
	>
		{isStarted ? 'AUTONOMY IN PROGRESS' : 'START'}
	</Button>

	<!-- STOP -->
	<Button
		class="relative w-full font-bold bg-red-600 text-white overflow-hidden"
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
			{isHoldingStop ? 'STOPPING AUTONOMY...' : 'HOLD TO STOP'}
		</span>
	</Button>
</div>
