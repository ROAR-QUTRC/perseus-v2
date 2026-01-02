<script lang="ts" module>
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'StateViewer';
	export const description = 'Subscribe to /cmd_vel_out (TwistStamped) and display speed on a gauge.';
	export const group = 'ROS';
	export const isRosDependent = true;

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			display: {
				maxSpeed: { type: 'number', value: 0.5, min: 0.001, max: 10, step: 0.01 }, // m/s
				unit: {
					type: 'select',
					value: 'm/s',
					options: [
						{ label: 'm/s', value: 'm/s' },
						{ label: 'km/h', value: 'km/h' }
					]
				},
				useMagnitude: { type: 'boolean', value: false }, // if true: sqrt(x^2+y^2+z^2)
				smoothing: { type: 'number', value: 0.2, min: 0.0, max: 0.95, step: 0.05 }, // EMA alpha
				showAngularZ: { type: 'boolean', value: true }
			}
		}
	});
</script>

<script lang="ts">
	import { onDestroy, onMount } from 'svelte';
	import * as ROSLIB from 'roslib';
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte';

	// ✅ Svelte 5 runes: do NOT destructure props if you want updates
	const props = $props<{ settings: any }>();

	let speed = $state(0); // displayed speed (after smoothing)
	let rawSpeed = $state(0);
	let angZ = $state(0);

	let status = $state<'connecting' | 'subscribed' | 'no_ros' | 'error'>('connecting');
	let errMsg = $state('');

	let topic: ROSLIB.Topic | null = null;
	let retryTimer: any = null;

	// for "last msg age" so you can see if it's actually updating
	let lastMsgMs = $state(0);
	let nowMs = $state(Date.now());
	let tickTimer: any = null;

	function toDisplayUnit(v_mps: number) {
		return props.settings?.groups?.display?.unit?.value === 'km/h' ? v_mps * 3.6 : v_mps;
	}

	function unitLabel() {
		return props.settings?.groups?.display?.unit?.value ?? 'm/s';
	}

	function computeSpeed(msg: any) {
		const useMag = !!props.settings?.groups?.display?.useMagnitude?.value;

		// Try multiple possible message structures
		let lx = msg?.twist?.linear?.x ?? msg?.linear?.x ?? 0;
		let ly = msg?.twist?.linear?.y ?? msg?.linear?.y ?? 0;
		let lz = msg?.twist?.linear?.z ?? msg?.linear?.z ?? 0;

		return useMag ? Math.sqrt(lx * lx + ly * ly + lz * lz) : Math.abs(lx);
	}

	function smooth(next: number) {
		const a = Number(props.settings?.groups?.display?.smoothing?.value ?? 0.2);
		if (a <= 0) return next; // no smoothing
		return speed + a * (next - speed); // EMA
	}

	function cleanup() {
		try {
			topic?.unsubscribe();
		} catch {}
		topic = null;

		if (retryTimer) {
			clearInterval(retryTimer);
			retryTimer = null;
		}

		if (tickTimer) {
			clearInterval(tickTimer);
			tickTimer = null;
		}
	}

	async function trySubscribe() {
		// keep retry timer running, but clear existing subscription
		try {
			topic?.unsubscribe();
		} catch {}
		topic = null;

		status = 'connecting';
		errMsg = '';

		let ros: any = null;
		try {
			ros = await Promise.resolve(getRosConnection() as any);
		} catch (e: any) {
			status = 'error';
			errMsg = String(e?.message ?? e);
			return;
		}

		if (!ros) {
			status = 'no_ros';
			return;
		}

		// surface connection issues if they happen
		try {
			ros.on('error', (e: any) => {
				status = 'error';
				errMsg = `ros error: ${String(e?.message ?? e)}`;
			});
			ros.on('close', () => {
				status = 'no_ros';
				errMsg = 'ros connection closed';
			});
		} catch {}

		try {
			// ✅ roslibjs/rosbridge expects ROS1-style type strings
			console.log('[StateViewer] Creating topic subscription to /cmd_vel_out (TwistStamped)');
			topic = new ROSLIB.Topic({
				ros,
				name: '/cmd_vel_out',
				messageType: 'geometry_msgs/TwistStamped'
			});

			// ✅ show "subscribed" immediately; don't wait for first message
			status = 'subscribed';
			lastMsgMs = 0;
			console.log('[StateViewer] Topic created, subscribing...');

			topic.subscribe((msg: any) => {
				console.log('[StateViewer] GOT MESSAGE!', msg);
				lastMsgMs = Date.now();

				const speed_val = computeSpeed(msg);
				console.log('[StateViewer] Computed speed:', speed_val);
				rawSpeed = speed_val;
				speed = smooth(rawSpeed);
				console.log('[StateViewer] After smooth, speed:', speed);

				angZ = msg?.twist?.angular?.z ?? 0;
			});

			console.log('[StateViewer] Subscription successful');

			// update "age" display
			if (!tickTimer) {
				tickTimer = setInterval(() => (nowMs = Date.now()), 200);
			}
		} catch (e: any) {
			console.error('[StateViewer] Error during topic subscription:', e);
			status = 'error';
			errMsg = String(e?.message ?? e);
		}
	}

	onMount(() => {
		trySubscribe();

		// Re-try occasionally in case ROS connects after widget mounts
		retryTimer = setInterval(() => {
			// if not yet subscribed or ros was down, keep trying
			if (status !== 'subscribed') trySubscribe();
		}, 1200);
	});

	onDestroy(() => cleanup());

	// ---- Gauge math ----
	const R = 108; // radius
	const C = 2 * Math.PI * R;

	// We’ll show ~270° of arc (like a speedometer)
	const ARC_FRAC = 0.75; // 75% of full circle
	const ARC_LEN = C * ARC_FRAC;
	const ARC_GAP = C - ARC_LEN;

	let maxSpeed = $derived(Math.max(0.1, Number(props.settings?.groups?.display?.maxSpeed?.value ?? 0.2)));
	let displayValue = $derived(toDisplayUnit(speed));
	let displayMax = $derived(toDisplayUnit(maxSpeed));

	let frac = $derived(Math.max(0, Math.min(1, speed / maxSpeed)));
	let dash = $derived(`${ARC_LEN * frac} ${C}`);

	let ageSec = $derived(lastMsgMs ? (nowMs - lastMsgMs) / 1000 : Infinity);
</script>

<style>
	.card {
		width: 100%;
		height: 100%;
		min-width: 200px;
		min-height: 180px;
		border-radius: 18px;
		padding: 18px;
		background: radial-gradient(120% 120% at 20% 10%, rgba(255, 255, 255, 0.1), rgba(0, 0, 0, 0.2));
		backdrop-filter: blur(6px);
		border: 1px solid rgba(255, 255, 255, 0.08);
		color: rgba(255, 255, 255, 0.92);
		display: grid;
		place-items: center;
	}

	/* ✅ SVG text uses fill, not color */
	svg text {
		fill: rgba(255, 255, 255, 0.92);
	}

	.label {
		font-size: 12px;
		letter-spacing: 0.18em;
		opacity: 0.85;
	}

	.value {
		font-size: 56px;
		font-weight: 700;
		line-height: 1;
		letter-spacing: 0.02em;
	}

	.unit {
		font-size: 12px;
		letter-spacing: 0.22em;
		opacity: 0.85;
	}

	.sub {
		margin-top: 8px;
		font-size: 12px;
		opacity: 0.75;
	}

	.badge {
		margin-top: 10px;
		font-size: 12px;
		opacity: 0.9;
		color: rgba(255, 255, 255, 0.9);
		padding: 6px 10px;
		border-radius: 999px;
		border: 1px solid rgba(255, 255, 255, 0.12);
		background: rgba(0, 0, 0, 0.18);
	}

	svg {
		overflow: visible;
	}

	.arc-bg {
		stroke: rgba(255, 255, 255, 0.18);
		stroke-width: 10;
		fill: none;
		stroke-linecap: round;
		stroke-dasharray: var(--arc-len) var(--c);
		stroke-dashoffset: var(--arc-gap);
	}

	.arc-fg {
		stroke: rgba(120, 255, 170, 0.95);
		stroke-width: 10;
		fill: none;
		stroke-linecap: round;
		filter: drop-shadow(0 0 6px rgba(120, 255, 170, 0.25));
	}

	.dot {
		fill: rgba(255, 255, 255, 0.9);
		opacity: 0.9;
	}
</style>

<div class="card" style="--c:{C}px; --arc-len:{ARC_LEN}px; --arc-gap:{ARC_GAP}px;">
	<div style="position: relative; width: 100%; height: 100%; display: flex; align-items: center; justify-content: center; min-width: 200px; min-height: 180px;">
		<svg style="width: 100%; height: 100%; max-width: 400px; max-height: 340px;" viewBox="0 0 260 220" aria-label="speedometer">
			<!-- Center -->
			<g transform="translate(130,120) rotate(135)">
				<!-- Background arc (270deg) -->
				<circle class="arc-bg" r={R} cx="0" cy="0" />

				<!-- Foreground arc -->
				<circle class="arc-fg" r={R} cx="0" cy="0" stroke-dasharray={dash} stroke-dashoffset={ARC_GAP} />
			</g>

			<!-- Text -->
			<g transform="translate(130,120)" text-anchor="middle">
				<text y="-52" class="label">/cmd_vel_out</text>
				<text y="8" class="value">{speed.toFixed(2)}</text>
				<text y="40" class="unit">{unitLabel()}</text>

				{#if props.settings?.groups?.display?.showAngularZ?.value}
					<text y="66" class="sub">ωz {angZ.toFixed(2)} rad/s</text>
				{/if}
			</g>
		</svg>

		<!-- Status -->
		<div class="badge" style="position:absolute; left: 10px; bottom: 2px;">
			{#if status === 'subscribed'}
				{#if ageSec === Infinity}
					Subscribed • waiting for msgs…
				{:else if ageSec > 1}
					Subscribed • last msg {ageSec.toFixed(1)}s ago
				{:else}
					/cmd_vel_out • max {displayMax.toFixed(1)} {unitLabel()}
				{/if}
			{:else if status === 'no_ros'}
				No ROS connection
			{:else if status === 'connecting'}
				Connecting…
			{:else}
				Error: {errMsg}
			{/if}
		</div>
	</div>
</div>
