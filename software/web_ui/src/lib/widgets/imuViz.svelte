<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'IMU Visualizer - 3D';
	// These properties are optional
	export const description = 'Visulaise IMU data with a 3D rover model';
	export const group: WidgetGroupType = 'ROS';
	export const isRosDependent = true; // Set to true if the widget requires a ROS connection

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/ros-bridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import ROSLIB from 'roslib';

	import { Canvas } from '@threlte/core';
	// import Rover from './imuViz/rover.svelte';
	import { onMount, untrack } from 'svelte';

	const smoothRandom = (absRange: number, currentValue: number): number => {
		const randomValue = Math.random() * absRange - absRange / 2;
		return currentValue + randomValue * 0.1;
	};

	$effect(() => {
		const ros = getRosConnection();

		untrack(() => {
			if (ros) {
				console.log('Subscribing to IMU data');
			} else {
				console.log('No ROS connection available');
			}
		});
	});

	let dummyIMU = $state<{
		angular_velocity: { x: number; y: number; z: number };
		linear_acceleration: { x: number; y: number; z: number };
	}>({
		angular_velocity: { x: 0, y: 0, z: 0 },
		linear_acceleration: { x: 0, y: 0, z: 0 }
	});

	onMount(() => {
		let count = 0;
		const intervalHandle = setInterval(() => {
			dummyIMU = {
				angular_velocity: {
					x: smoothRandom(0.1, dummyIMU.angular_velocity.x),
					y: smoothRandom(0.1, dummyIMU.angular_velocity.y),
					z: smoothRandom(0.1, dummyIMU.angular_velocity.z)
					// x: 0,
					// y: 0,
					// z: 0
				},
				linear_acceleration: {
					x: smoothRandom(0.1, dummyIMU.linear_acceleration.x),
					y: smoothRandom(0.1, dummyIMU.linear_acceleration.y),
					z: smoothRandom(0.1, dummyIMU.linear_acceleration.z)
					// x: Math.sin(count / 100) * 0.5,
					// y: 0.5,
					// z: Math.cos(count / 100) * 0.5
				}
			};
			count++;
		}, 5);
		return () => {
			clearTimeout(intervalHandle);
		};
	});

	let rotation: [x: number, y: number, z: number] = $derived([
		dummyIMU.angular_velocity.x,
		dummyIMU.angular_velocity.y,
		dummyIMU.angular_velocity.z
	]);
	let direction: [x: number, y: number, z: number] = $derived([
		dummyIMU.linear_acceleration.x,
		dummyIMU.linear_acceleration.y,
		dummyIMU.linear_acceleration.z
	]);
</script>

<div class="relative h-full w-full">
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
	<Canvas>
		<!-- <Rover {direction} {rotation} position={[0, 0, 0]} /> -->
	</Canvas>
</div>
