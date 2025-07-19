<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'IMU Visualiser - 2D';
	// These properties are optional
	export const description = 'Visulaise IMU data with a 3D rover model';
	export const group: WidgetGroupType = 'ROS';
	export const isRosDependent = true; // Set to true if the widget requires a ROS connection

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			general: {
				showRawData: {
					type: 'switch',
					description: 'Show raw IMU data as an overlay',
					value: 'true'
				}
			}
		}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/ros-bridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import ROSLIB from 'roslib';
	import { onMount } from 'svelte';
	import { updateHUD, type ImuDataType } from './imuViz/hud';

	const smoothRandom = (absRange: number, currentValue: number): number => {
		const randomValue = Math.random() * absRange - absRange / 2;
		return currentValue + randomValue * 0.1;
	};

	let dummyIMU = $state<ImuDataType>({
		angular_velocity: { x: 0, y: 0, z: 0 },
		linear_acceleration: { x: 0, y: 0, z: 0 }
	});

	let canvas = $state<HTMLCanvasElement>();

	onMount(() => {
		const ctx = canvas?.getContext('2d');

		if (!ctx || !canvas) {
			console.error('Canvas context or element not found');
			return;
		}

		let count = 0;
		const useTrig = true;
		const intervalHandle = setInterval(() => {
			if (useTrig) {
				dummyIMU = {
					angular_velocity: {
						x: Math.sin(count / 100) / 2,
						y: 0,
						z: Math.cos(count / 100) / 2
					},
					linear_acceleration: {
						x: Math.sin(count / 10) / 2,
						y: 0.5,
						z: Math.cos(count / 10) / 2
					}
				};
				count++;
			} else {
				dummyIMU = {
					angular_velocity: {
						x: smoothRandom(0.1, dummyIMU.angular_velocity.x),
						y: smoothRandom(0.1, dummyIMU.angular_velocity.y),
						z: smoothRandom(0.1, dummyIMU.angular_velocity.z)
					},
					linear_acceleration: {
						x: smoothRandom(0.1, dummyIMU.linear_acceleration.x),
						y: smoothRandom(0.1, dummyIMU.linear_acceleration.y),
						z: smoothRandom(0.1, dummyIMU.linear_acceleration.z)
					}
				};
			}
			updateHUD(ctx, canvas!, dummyIMU);
		}, 1 / 100);

		return () => {
			clearTimeout(intervalHandle);
		};
	});

	export const ro = (node: Element, callback: (entry: ResizeObserverEntry) => {}) => {
		const ro = new ResizeObserver(([entry]) => callback(entry));
		ro.observe(node);
		return {
			destroy: () => ro.disconnect()
		};
	};

	const resizeCanvas = ({ contentRect }: ResizeObserverEntry) => {
		if (!canvas) {
			console.error('Canvas element not found');
			return {};
		}
		canvas.width = contentRect.width;
		canvas.height = contentRect.height;
		const ctx = canvas.getContext('2d');
		if (!ctx) {
			console.error('Canvas context not found');
			return {};
		}
		updateHUD(ctx, canvas, dummyIMU);
		return {}; // Fixes typescript error
	};
</script>

<div class="relative h-full w-full">
	{#if settings.groups.general.showRawData.value === 'true'}
		<div class="bg-card absolute left-0 top-0 z-10 rounded-lg border bg-opacity-70 p-4">
			<p>angular_velocity</p>
			<ul>
				<li>x: {dummyIMU.angular_velocity.x.toFixed(4)}</li>
				<li>y: {dummyIMU.angular_velocity.y.toFixed(4)}</li>
				<li>z: {dummyIMU.angular_velocity.z.toFixed(4)}</li>
			</ul>
			<p>linear_acceleration</p>
			<ul>
				<li>x: {dummyIMU.linear_acceleration.x.toFixed(4)}</li>
				<li>y: {dummyIMU.linear_acceleration.y.toFixed(4)}</li>
				<li>z: {dummyIMU.linear_acceleration.z.toFixed(4)}</li>
			</ul>
		</div>
	{/if}
	<canvas bind:this={canvas} use:ro={resizeCanvas} class="h-full w-full"></canvas>
</div>
