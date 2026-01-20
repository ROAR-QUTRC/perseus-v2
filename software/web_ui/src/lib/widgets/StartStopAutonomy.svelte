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
				jsonFile: {
					type: 'select',
					value: '/waypoints_simulation.json',
					options: [
						{ label: 'waypoints.json', value: '/waypoints.json' },
						{ label: 'waypoints_simulation.json', value: '/waypoints_simulation.json' },
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
	import { browser } from '$app/environment';

	// ---------------- ROS (Services) ----------------
	let runSrv = $state<ROSLIB.Service | null>(null);
	let cancelSrv = $state<ROSLIB.Service | null>(null);

	// ---------------- UI state ----------------
	let isStarted = $state(false);

	// Service status / busy
	let srvStatus = $state<string>('ROS disconnected');
	let srvBusy = $state(false);

	// Navigation info subscription (internal, not exposed as state)
	let navInfoSub: ROSLIB.Topic | null = null;

	// ---------------- Hold-to-stop ----------------
	const HOLD_MS = 3000;
	const PROGRESS_UPDATE_INTERVAL_MS = 16; // ~60fps
	let isHoldingStop = $state(false);
	let holdProgress = $state(0);
	let holdTimer: number | null = null;
	let progressTimer: number | null = null;
	let holdStartTs = 0;

	const clearHold = () => {
		isHoldingStop = false;
		holdProgress = 0;

		if (holdTimer) globalThis.clearTimeout(holdTimer as unknown as number);
		if (progressTimer) globalThis.clearInterval(progressTimer as unknown as number);

		holdTimer = null;
		progressTimer = null;
	};

	// ---------------- Waypoints ----------------
	type Waypoint = { name: string; x: number; y: number; yaw?: number };

	let waypoints = $state<Waypoint[]>([]);
	let jsonError = $state<string | null>(null);
	let isLoadingYaml = $state(false);

	// Selected JSON HTTP path (used for preview table fetch)
	const getSelectedJson = () => settings.groups.waypoints.jsonFile.value || '/waypoints.json';

	const getSelectedJsonBasename = () => {
		const p = getSelectedJson();
		const last = p.split('/').pop();
		return last ?? 'waypoints.json';
	};

	const loadWaypointsJson = async () => {
		try {
			isLoadingYaml = true;
			jsonError = null;

			const url = getSelectedJson(); // HTTP path
			const res = await fetch(url, { cache: 'no-store' });
			if (!res.ok) throw new Error(`Failed to fetch ${url}`);

			const loaded = await res.json();
			if (!Array.isArray(loaded)) {
				throw new Error('JSON must be an array of waypoints');
			}

			waypoints = loaded
				.map((w: any, i: number) => ({
					name: w.name ?? `WP${i + 1}`,
					x: Number(w.x),
					y: Number(w.y),
					yaw: w.yaw || undefined ? Number(w.yaw) : undefined // if the yaw is provided
				}))
				.filter((w) => Number.isFinite(w.x) && Number.isFinite(w.y));

			if (waypoints.length === 0) {
				throw new Error('No valid waypoints found in JSON');
			}
		} catch (e: any) {
			jsonError = e?.message ?? 'Failed to load JSON';
			waypoints = [];
		} finally {
			isLoadingYaml = false;
		}
	};

	const saveWaypointsJsonFile = () => {
		if (waypoints.length === 0) {
			jsonError = 'No waypoints to save';
			return;
		}

		const json = JSON.stringify(waypoints, null, 2);
		const blob = new Blob([json], { type: 'application/json' });
		const url = URL.createObjectURL(blob);
		const a = document.createElement('a');
		a.href = url;
		a.download = `waypoints_${new Date().toISOString().split('T')[0]}.json`;
		document.body.appendChild(a);
		a.click();
		document.body.removeChild(a);
		URL.revokeObjectURL(url);
	};

	const loadWaypointsJsonFileUpload = async (event: Event) => {
		const target = event.target as HTMLInputElement;
		const files = target.files;
		if (!files || files.length === 0) return;

		try {
			isLoadingYaml = true;
			jsonError = null;

			const file = files[0];
			const text = await file.text();
			const loaded = JSON.parse(text);

			if (!Array.isArray(loaded)) {
				throw new Error('JSON must be an array of waypoints');
			}

			waypoints = loaded
				.map((w: any, i: number) => ({
					name: w.name ?? `WP${i + 1}`,
					x: Number(w.x),
					y: Number(w.y),
					yaw: w.yaw !== undefined ? Number(w.yaw) : undefined
				}))
				.filter((w) => Number.isFinite(w.x) && Number.isFinite(w.y));

			if (waypoints.length === 0) {
				throw new Error('No valid waypoints in JSON file');
			}

			srvStatus = `Loaded ${waypoints.length} waypoints from JSON`;
		} catch (e: any) {
			jsonError = e?.message ?? 'Failed to load JSON';
			waypoints = [];
		} finally {
			isLoadingYaml = false;
			target.value = ''; // reset file input
		}
	};

	// Auto-reload when dropdown changes
	$effect(() => {
		void settings.groups.waypoints.jsonFile.value;
		loadWaypointsJson();
	});

	// ---------------- ROS connection (services) ----------------
	$effect(() => {
		const ros = getRosConnection();
		if (!ros) {
			runSrv = null;
			cancelSrv = null;
			srvStatus = 'ROS disconnected';
			srvBusy = false;
			isStarted = false;
			clearHold();
			return;
		}

		runSrv = new ROSLIB.Service({
			ros,
			name: '/autonomy/run_waypoints',
			serviceType: 'perseus_interfaces/srv/RunWaypoints'
		});

		cancelSrv = new ROSLIB.Service({
			ros,
			name: '/autonomy/cancel_waypoints',
			serviceType: 'perseus_interfaces/srv/RunWaypoints'
		});

		srvStatus = 'ROS connected';
	});

	const sendStart = () => {
		if (!runSrv) {
			srvStatus = 'Run service not ready';
			return;
		}

		if (waypoints.length === 0) {
			srvStatus = 'No waypoints loaded';
			return;
		}

		srvBusy = true;
		srvStatus = 'Sending run request...';

		runSrv.callService(
			new ROSLIB.ServiceRequest({ 
				waypoints: waypoints.map((wp) => ({
					name: wp.name,
					x: wp.x,
					y: wp.y,
					yaw: wp.yaw ?? 0.0
				}))
			}),
			(resp: any) => {
				srvBusy = false;
				srvStatus = resp?.message ?? 'Run response received';
				if (resp?.success) isStarted = true;
			},
			(err: any) => {
				srvBusy = false;
				srvStatus = `Run failed: ${err?.toString?.() ?? err}`;
			}
		);
	};

	const startHoldStop = () => {
		if (!isStarted || isHoldingStop || srvBusy) return;

		isHoldingStop = true;
		holdStartTs = Date.now();

		// NOTE: use globalThis timers (SSR-safe) + cast for TS
		holdTimer = globalThis.setTimeout(() => {
			if (!cancelSrv) {
				srvStatus = 'Cancel service not ready';
				isStarted = false;
				clearHold();
				return;
			}

			srvBusy = true;
			srvStatus = 'Cancelling...';

			cancelSrv.callService(
				new ROSLIB.ServiceRequest({}),
				(resp: any) => {
					srvBusy = false;
					srvStatus = resp?.message ?? 'Cancel response received';
					isStarted = false;
					clearHold();
				},
				(err: any) => {
					srvBusy = false;
					srvStatus = `Cancel failed: ${err?.toString?.() ?? err}`;
					clearHold();
				}
			);
		}, HOLD_MS) as unknown as number;

		progressTimer = globalThis.setInterval(() => {
			holdProgress = Math.min(1, (Date.now() - holdStartTs) / HOLD_MS);
		}, PROGRESS_UPDATE_INTERVAL_MS) as unknown as number;
	};

	// Set up persistent subscription to navigation_info
	$effect(() => {
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

	onMount(() => {
		loadWaypointsJson();
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
						<p class="text-xs opacity-60">{getSelectedJson()}</p>

						<select
							class="h-7 rounded-md border bg-background px-2 text-xs hover:bg-accent"
							bind:value={settings.groups.waypoints.jsonFile.value}
						>
							{#each settings.groups.waypoints.jsonFile.options as opt}
								<option value={opt.value}>{opt.label}</option>
							{/each}
						</select>

						<button
							class="rounded-md border px-2 py-1 text-xs hover:bg-accent"
							onclick={loadWaypointsJson}
						>
							Refresh
						</button>

						<button
							class="rounded-md border px-2 py-1 text-xs hover:bg-accent"
							onclick={saveWaypointsJsonFile}
							disabled={waypoints.length === 0}
						>
							Save JSON
						</button>

						<label class="rounded-md border px-2 py-1 text-xs hover:bg-accent cursor-pointer">
							<input
								type="file"
								accept=".json"
								onchange={loadWaypointsJsonFileUpload}
								class="hidden"
							/>
							Load JSON
						</label>

						{#if isLoadingYaml}
							<span class="text-xs animate-pulse opacity-70">Loading…</span>
						{/if}
					</div>
				</div>
			</div>

			{#if jsonError}
				<div class="px-3 py-2">
					<p class="text-xs font-semibold text-red-500">{jsonError}</p>
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
			{#if srvBusy && !isStarted}
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
			disabled={!isStarted || !cancelSrv || srvBusy}
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
			Status: <span class="font-mono">{srvStatus}</span>
		</p>
	</div>
</div>