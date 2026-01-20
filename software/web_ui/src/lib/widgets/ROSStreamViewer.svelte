<script lang="ts" module>
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Image Stream';
	export const description = 'View a live image topic (sensor_msgs/Image).';
	export const group: WidgetGroupType = 'ROS';
	export const isRosDependent = true;

	export const settings: WidgetSettingsType = $state({
		groups: {
			stream: {
				topic: {
					type: 'string',
					value: '/camera/image'
				}
			}
		}
	});
</script>

<script lang="ts">
	import { onDestroy } from 'svelte';
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte';
	import * as ROSLIB from 'roslib';

	let imgSrc = $state<string>('');
	let status = $state<string>('ROS disconnected');

	let sub: ROSLIB.Topic | null = null;
	let canvas: HTMLCanvasElement | null = null;

	const stop = () => {
		if (sub) {
			try { sub.unsubscribe(); } catch {}
		}
		sub = null;
	};

	// Utility: base64 → Uint8Array
	function base64ToBytes(b64: string): Uint8Array {
		const bin = atob(b64);
		const out = new Uint8Array(bin.length);
		for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
		return out;
	}

	$effect(() => {
		const ros = getRosConnection();
		const topicName = settings.groups.stream.topic.value?.trim();

		stop();

		if (!ros) {
			status = 'ROS disconnected';
			return;
		}
		if (!topicName) {
			status = 'No topic set';
			return;
		}

		status = `Subscribing: ${topicName}`;

		sub = new ROSLIB.Topic({
			ros,
			name: topicName,
			messageType: 'sensor_msgs/msg/Image'
		});

		sub.subscribe((msg: any) => {
			const width = msg.width;
			const height = msg.height;
			const encoding = String(msg.encoding).toLowerCase();

			if (!canvas) {
				canvas = document.createElement('canvas');
			}
			canvas.width = width;
			canvas.height = height;

			const ctx = canvas.getContext('2d');
			if (!ctx) return;

			let bytes: Uint8Array;

			if (typeof msg.data === 'string') {
				// rosbridge base64
				bytes = base64ToBytes(msg.data);
			} else {
				// rosbridge array<number>
				bytes = new Uint8Array(msg.data);
			}

			const imgData = ctx.createImageData(width, height);
			const dst = imgData.data;

			if (encoding === 'rgb8') {
				for (let i = 0, j = 0; i < bytes.length; i += 3, j += 4) {
					dst[j]     = bytes[i];
					dst[j + 1] = bytes[i + 1];
					dst[j + 2] = bytes[i + 2];
					dst[j + 3] = 255;
				}
			} else if (encoding === 'bgr8') {
				for (let i = 0, j = 0; i < bytes.length; i += 3, j += 4) {
					dst[j]     = bytes[i + 2];
					dst[j + 1] = bytes[i + 1];
					dst[j + 2] = bytes[i];
					dst[j + 3] = 255;
				}
			} else {
				status = `Unsupported encoding: ${encoding}`;
				return;
			}

			ctx.putImageData(imgData, 0, 0);
			imgSrc = canvas.toDataURL('image/png');
			status = `Streaming: ${topicName}`;
		});
	});

	onDestroy(() => stop());
</script>

<div class="flex h-full w-full flex-col gap-2">
	<div class="text-xs opacity-70">{status}</div>

	<div class="min-h-0 flex-1 overflow-hidden rounded-xl border bg-black/30">
		{#if imgSrc}
			<img
				src={imgSrc}
				alt="ROS image stream"
				class="h-full w-full object-contain"
				draggable="false"
			/>
		{:else}
			<div class="flex h-full w-full items-center justify-center text-xs opacity-60">
				No frames yet
			</div>
		{/if}
	</div>
</div>
