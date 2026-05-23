<script lang="ts" module>
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'SLAM Map';
	export const description =
		'Display a nav_msgs/OccupancyGrid map topic (e.g. /map) with robot pose + Nav2 path overlay.';
	export const group: WidgetGroupType = 'ROS';
	export const isRosDependent = true;

	export const settings: WidgetSettingsType = $state({
		groups: {
			map: {
				topic: { type: 'string', value: '/map' }
			},
			robot: {
				topic: { type: 'string', value: '/autonomy/navigation_info' },
				messageType: { type: 'string', value: 'perseus_interfaces/msg/NavigationData' },
				markerSize: {
					type: 'number',
					description: 'Marker size (base scale in pixels)',
					value: 2
				}
			},
			path: {
				topic: { type: 'string', value: '/plan' },
				messageType: { type: 'string', value: 'nav_msgs/msg/Path' }
			}
		}
	});
</script>

<script lang="ts">
	import { onDestroy } from 'svelte';
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte';
	import * as ROSLIB from 'roslib';

	let status = $state<string>('ROS disconnected');

	let canvasEl: HTMLCanvasElement | null = null;
	let ctx: CanvasRenderingContext2D | null = null;

	let mapSub: ROSLIB.Topic | null = null;
	let poseSub: ROSLIB.Topic | null = null;
	let pathSub: ROSLIB.Topic | null = null;

	let lastMapMsg: any = null;
	let lastRobotPose: { x: number; y: number; yaw: number } | null = null;
	let lastPathMsg: any = null;

	let mapCanvas: HTMLCanvasElement | null = null;
	let mapCtx: CanvasRenderingContext2D | null = null;

	const stop = () => {
		if (mapSub) try { mapSub.unsubscribe(); } catch {}
		if (poseSub) try { poseSub.unsubscribe(); } catch {}
		if (pathSub) try { pathSub.unsubscribe(); } catch {}
		mapSub = null;
		poseSub = null;
		pathSub = null;
	};

	function base64ToBytes(b64: string): Uint8Array {
		const bin = atob(b64);
		const out = new Uint8Array(bin.length);
		for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
		return out;
	}

	function quatToYaw(q: { x: number; y: number; z: number; w: number }): number {
		const siny = 2 * (q.w * q.z + q.x * q.y);
		const cosy = 1 - 2 * (q.y * q.y + q.z * q.z);
		return Math.atan2(siny, cosy);
	}

	function ensureContexts() {
		if (!canvasEl) return false;
		if (!ctx) ctx = canvasEl.getContext('2d');
		if (!ctx) return false;

		if (!mapCanvas) mapCanvas = document.createElement('canvas');
		if (!mapCtx) mapCtx = mapCanvas.getContext('2d');

		return !!mapCtx;
	}

	function drawMapToOffscreen(msg: any) {
		if (!ensureContexts()) return;

		const info = msg.info;
		const w = info?.width ?? 0;
		const h = info?.height ?? 0;
		if (!w || !h) return;

		let data: Int8Array;
		if (typeof msg.data === 'string') {
			const u8 = base64ToBytes(msg.data);
			// Use exact length to avoid drawing garbage if buffer is larger
			data = new Int8Array(u8.buffer, u8.byteOffset, u8.byteLength);
		} else {
			data = Int8Array.from(msg.data);
		}

		// Rotate 90 degrees: swap width and height
		mapCanvas!.width = h;
		mapCanvas!.height = w;
		canvasEl!.width = h;
		canvasEl!.height = w;

		const img = mapCtx!.createImageData(h, w);
		const px = img.data;

		// Rotate pixels 90 degrees counter-clockwise + horizontal flip (matching your worldToPixel)
		for (let y = 0; y < h; y++) {
			for (let x = 0; x < w; x++) {
				const v = data[x + y * w];
				const c = v < 0 ? 40 : v === 0 ? 220 : 0;

				const rx = h - 1 - y;
				const ry = w - 1 - x;

				const i = (rx + ry * h) * 4;
				px[i] = px[i + 1] = px[i + 2] = c;
				px[i + 3] = 255;
			}
		}

		mapCtx!.putImageData(img, 0, 0);
	}

	function worldToPixel(x: number, y: number, info: any) {
		const res = info?.resolution;
		const origin = info?.origin?.position;
		if (!res || !origin) return null;

		const gx = (x - origin.x) / res;
		const gy = (y - origin.y) / res;

		const w = info.width;
		const h = info.height;

		// Apply 90-degree counter-clockwise rotation with horizontal flip: (gx, gy) -> (h - 1 - gy, w - 1 - gx)
		const px = h - 1 - gy;
		const py = w - 1 - gx;

		return { px, py };
	}

	function drawPathOverlay() {
		if (!ensureContexts() || !lastPathMsg || !lastMapMsg) return;

		const poses = lastPathMsg?.poses;
		if (!Array.isArray(poses) || poses.length < 2) return;

		ctx!.strokeStyle = 'rgba(0, 150, 255, 0.7)';
		ctx!.lineWidth = 2;
		ctx!.lineCap = 'round';
		ctx!.lineJoin = 'round';

		ctx!.beginPath();
		let firstPoint = true;

		for (const poseStamped of poses) {
			const pos = poseStamped?.pose?.position;
			if (!pos) continue;

			const p = worldToPixel(pos.x, pos.y, lastMapMsg.info);
			if (!p) continue;

			if (firstPoint) {
				ctx!.moveTo(p.px, p.py);
				firstPoint = false;
			} else {
				ctx!.lineTo(p.px, p.py);
			}
		}

		if (!firstPoint) ctx!.stroke();
	}

	function drawRobotMarker() {
		if (!ensureContexts() || !lastMapMsg || !lastRobotPose) return;

		const p = worldToPixel(lastRobotPose.x, lastRobotPose.y, lastMapMsg.info);
		if (!p) return;

		const base = settings.groups.robot.markerSize.value;
		const r = Math.max(2, base * 0.4);
		const arrowLen = Math.max(6, base * 1.4);
		const arrowW = Math.max(2, base * 0.3);
		const outline = Math.max(1, base * 0.12);

		// Adjust yaw for 90-degree CCW rotation (+ mirror flip)
		const ang = -lastRobotPose.yaw - Math.PI / 2;

		ctx!.save();
		ctx!.translate(p.px, p.py);
		ctx!.rotate(ang);

		// Dot
		ctx!.beginPath();
		ctx!.arc(0, 0, r, 0, Math.PI * 2);
		ctx!.fillStyle = 'rgba(255,60,60,0.95)';
		ctx!.fill();
		ctx!.lineWidth = outline;
		ctx!.strokeStyle = 'rgba(0,0,0,0.6)';
		ctx!.stroke();

		// Arrow
		ctx!.beginPath();
		ctx!.moveTo(arrowLen, 0);
		ctx!.lineTo(arrowLen - r * 2, -arrowW);
		ctx!.lineTo(arrowLen - r * 2, arrowW);
		ctx!.closePath();
		ctx!.fill();
		ctx!.stroke();

		ctx!.restore();
	}

	// Single render path: map -> path -> robot
	function render() {
		if (!ensureContexts() || !lastMapMsg) return;

		ctx!.clearRect(0, 0, canvasEl!.width, canvasEl!.height);
		ctx!.drawImage(mapCanvas!, 0, 0);

		drawPathOverlay();
		drawRobotMarker();
	}

	$effect(() => {
		const ros = getRosConnection();
		const mapTopic = settings.groups.map.topic.value;
		const poseTopic = settings.groups.robot.topic.value;
		const poseType = settings.groups.robot.messageType.value;
		const pathTopic = settings.groups.path.topic.value;
		const pathType = settings.groups.path.messageType.value;

		stop();
		lastMapMsg = null;
		lastRobotPose = null;
		lastPathMsg = null;

		if (!ros) {
			status = 'ROS disconnected';
			return;
		}

		if (!mapTopic || !poseTopic || !poseType) {
			status = 'Missing topic configuration';
			return;
		}

		status = `Subscribing: ${mapTopic} + ${poseTopic}${pathTopic ? ' + ' + pathTopic : ''}`;

		mapSub = new ROSLIB.Topic({
			ros,
			name: mapTopic,
			messageType: 'nav_msgs/msg/OccupancyGrid'
		});

		poseSub = new ROSLIB.Topic({
			ros,
			name: poseTopic,
			messageType: poseType
		});

		if (pathTopic && pathType) {
			pathSub = new ROSLIB.Topic({
				ros,
				name: pathTopic,
				messageType: pathType
			});
		}

		mapSub.subscribe((msg) => {
			lastMapMsg = msg;
			drawMapToOffscreen(msg);
			render();
		});

		poseSub.subscribe((msg) => {
			const p = msg?.current_pose?.pose?.position;
			const q = msg?.current_pose?.pose?.orientation;
			if (!p || !q) return;

			lastRobotPose = { x: p.x, y: p.y, yaw: quatToYaw(q) };
			render();
		});

		pathSub?.subscribe((msg) => {
			lastPathMsg = msg;
			render();
		});
	});

	// Redraw when marker size changes (keeps path visible too)
	$effect(() => {
		void settings.groups.robot.markerSize.value;
		render();
	});

	onDestroy(() => stop());
</script>

<div class="flex h-full w-full flex-col gap-2">
	<div class="text-xs opacity-70">{status}</div>
	<div class="min-h-0 flex-1 overflow-hidden rounded-xl border bg-black/30">
		<canvas bind:this={canvasEl} class="h-full w-full object-contain"></canvas>
	</div>
</div>