<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Deployment Mechanism';
	// These properties are optional
	export const description = 'Control the deployment mechanism motor.';
	export const group: WidgetGroupType = 'ROS';
	export const isRosDependent = true; // Set to true if the widget requires a ROS connection

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			topics: {
				cmdTopic: {
					type: 'text',
					description: 'Topic to publish motor speed commands (-1.0 to 1.0)',
					value: '/deployment_mechanism/motor/cmd'
				}
			},
			control: {
				publishRate: {
					type: 'number',
					description: 'Publish rate in Hz',
					value: '10'
				},
				deadband: {
					type: 'number',
					description: 'Deadband as a fraction (e.g. 0.05 = 5%)',
					value: '0.05'
				}
			}
		}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import * as ROSLIB from 'roslib';
	import { onMount } from 'svelte';

	// ROS handle
	let cmdTopic = $state<ROSLIB.Topic | null>(null);

	// Slider state
	let sliderValue = $state(0);         // -1.0 to 1.0
	let isDragging = $state(false);
	let sliderParent = $state<HTMLDivElement | null>(null);

	// Publish interval
	let intervalHandle: NodeJS.Timeout | null = null;

	// Status
	let statusMessage = $state('ROS disconnected');

	// ROS2 connection management
	$effect(() => {
		const ros = getRosConnection();

		if (!ros) {
			cmdTopic = null;
			statusMessage = 'ROS disconnected';
			return;
		}

		cmdTopic = new ROSLIB.Topic({
			ros,
			name: settings.groups.topics.cmdTopic.value ?? '/deployment_mechanism/motor/cmd',
			messageType: 'std_msgs/Float32'
		});
		cmdTopic.advertise();
		statusMessage = 'ROS connected';
	});

	// Publish at configured rate
	$effect(() => {
		const rateHz = Number(settings.groups.control.publishRate.value) || 10;
		if (intervalHandle) clearInterval(intervalHandle);
		intervalHandle = setInterval(publishCmd, 1000 / rateHz);
		return () => { if (intervalHandle) clearInterval(intervalHandle); };
	});

	const deadband = (v: number) => {
		const db = Number(settings.groups.control.deadband.value) || 0.05;
		return Math.abs(v) < db ? 0 : v;
	};

	const publishCmd = () => {
		if (!cmdTopic) return;
		cmdTopic.publish({ data: deadband(sliderValue) });
	};

	// Horizontal drag logic
	const getSliderValue = (clientX: number) => {
		const rect = sliderParent?.getBoundingClientRect();
		if (!rect) return 0;
		const clamped = Math.min(Math.max(clientX, rect.left), rect.right);
		return ((clamped - rect.left) / rect.width) * 2 - 1;
	};

	const onPointerDown = (e: PointerEvent) => {
		e.preventDefault();
		isDragging = true;
		sliderValue = getSliderValue(e.clientX);
		(e.target as HTMLElement).setPointerCapture(e.pointerId);
	};

	const onPointerMove = (e: PointerEvent) => {
		if (!isDragging) return;
		e.preventDefault();
		sliderValue = getSliderValue(e.clientX);
	};

	const onPointerUp = (e: PointerEvent) => {
		e.preventDefault();
		isDragging = false;
		sliderValue = 0;
		publishCmd();
	};

	// Thumb horizontal position as CSS percent
	const thumbLeft = $derived(`calc(${(sliderValue + 1) * 50}% - 25px)`);

	// Slider colour. Orange when positive, purple when negative, grey at zero
	const trackColor = $derived(
		Math.abs(deadband(sliderValue)) < 0.01
			? 'bg-muted'
			: sliderValue > 0
				? 'bg-orange-500'
				: 'bg-purple-500'
	);

	onMount(() => {
		return () => {
			if (intervalHandle) clearInterval(intervalHandle);
			// Send zero on unmount
			if (cmdTopic) cmdTopic.publish({ data: 0 });
		};
	});
</script>

<div class="flex h-full w-full flex-col items-center justify-center gap-6 p-4">
	<!-- Speed readout -->
	<div class="flex flex-col items-center gap-1">
		<p class="text-4xl font-bold tabular-nums">
			{(deadband(sliderValue) * 100).toFixed(0)}<span class="text-xl opacity-60">%</span>
		</p>
		<p class="text-xs opacity-50">
			{sliderValue >= 0 ? 'Forward' : 'Reverse'}
		</p>
	</div>

	<!-- Slider track -->
	<div
		bind:this={sliderParent}
		class="relative h-[60px] w-full max-w-xs cursor-grab rounded-full opacity-70 {trackColor} touch-none select-none"
		onpointerdown={onPointerDown}
		onpointermove={onPointerMove}
		onpointerup={onPointerUp}
		onpointercancel={onPointerUp}
		role="slider"
		aria-valuenow={sliderValue}
		aria-valuemin={-1}
		aria-valuemax={1}
		tabindex="0"
	>
		<!-- Centre marker -->
		<div class="absolute inset-y-0 left-1/2 w-0.5 -translate-x-1/2 bg-white/30 rounded"></div>

		<!-- Thumb -->
		<div
			class="pointer-events-none absolute top-[5px] h-[50px] w-[50px] rounded-full bg-white shadow-lg transition-[left] duration-75"
			style:left={thumbLeft}
		></div>
	</div>

	<p class="text-xs opacity-40">Drag left / right · releases to centre</p>

	<!-- Status -->
	<p class="text-xs opacity-60">Status: <span class="font-mono">{statusMessage}</span></p>
</div>