<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Bucket Controller';
	// These properties are optional
	export const description = 'Control the E&C bucket via ROS';
	export const group: WidgetGroupType = 'ROS';
	export const isRosDependent = true; // Set to true if the widget requires a ROS connection

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			general: {
				masterSpeedMultiplier: {
					type: 'number',
					description: 'Base multiplier applied to all speeds',
					value: '1'
				}
			}
		}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/ros-bridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import ROSLIB from 'roslib';
	import { onMount } from 'svelte';

	// Widget logic goes here
	$effect(() => {
		const ros = getRosConnection();
		if (!ros) return;
	});

	// ---------------------------------------
	// 		  Joystick HTML components
	// ---------------------------------------
	const handles = $state<
		Record<string, { parent: HTMLDivElement | null; active: boolean; value: number }>
	>({
		lift: { parent: null, active: false, value: 0 },
		tilt: { parent: null, active: false, value: 0 },
		jaws: { parent: null, active: false, value: 0 },
		rotate: { parent: null, active: false, value: 0 }
	});

	const onStart = (event: PointerEvent, handle: string) => {
		event.preventDefault();

		handles[handle].active = true;
		console.log('Start', handle, event.clientY);
	};

	const onMove = (event: PointerEvent, handle: string) => {
		if (!handles[handle].active) return;

		event.preventDefault();

		const boundingRect = handles[handle].parent?.getBoundingClientRect();
		if (!boundingRect) return;

		// Thumb padding 5px + 25px radius
		const padding = 5 + 25;
		const clampedY = Math.round(
			Math.max(boundingRect.top + padding, Math.min(boundingRect.bottom - padding, event.clientY))
		);
		handles[handle].value = (clampedY - boundingRect.top) / boundingRect.height;

		console.log('Move', handle, event.clientY, boundingRect.top, boundingRect.bottom);
	};

	const onStop = (event: PointerEvent, handle: string) => {
		if (!handles[handle].active) return;

		event.preventDefault();

		handles[handle].value = 0;
		handles[handle].active = false;
		console.log('Stop', handle, event.clientY);
	};

	onMount(() => {
		Object.keys(handles).forEach((handle) => {
			if (handles[handle].parent) {
				handles[handle].parent?.addEventListener('pointerdown', (e) => onStart(e, handle));
				handles[handle].parent?.addEventListener('pointermove', (e) => onMove(e, handle));
				handles[handle].parent?.addEventListener('pointerup', (e) => onStop(e, handle));
				handles[handle].parent?.addEventListener('pointerleave', (e) => onStop(e, handle));
				const rect = handles[handle].parent.getBoundingClientRect();
				console.log('Initialised: ', rect);
			} else console.warn('Failed to initialise: ', handle);

			// handles[handle].element.onpointerdown = (event) => {
			// 	event.preventDefault();
			// 	let startY = event.clientY;
			// 	const onPointerMove = (moveEvent: PointerEvent) => {
			// 		moveEvent.preventDefault();
			// 		const deltaY = moveEvent.clientY - startY;
			// 		startY = moveEvent.clientY;
			// 		handles[handle].value -= deltaY * 0.1; // Adjust sensitivity as needed
			// 		handles[handle].value = Math.max(-100, Math.min(100, handles[handle].value)); // Clamp between -100 and 100
			// 	};
			// 	const onPointerUp = (upEvent: PointerEvent) => {
			// 		upEvent.preventDefault();
			// 		window.removeEventListener('pointermove', onPointerMove);
			// 		window.removeEventListener('pointerup', onPointerUp);
			// 	};
			// 	window.addEventListener('pointermove', onPointerMove);
			// 	window.addEventListener('pointerup', onPointerUp);
			// };
		});

		// Cleanup ros connection on unmount
		return () => {};
	});
</script>

<!-- Value is the height from -1 to 1 -->
{#snippet thumb(hidden: boolean, value: number)}
	{#if !hidden}
		<div
			style:top={`calc(${50 - value * 50}% - 25px)`}
			class={'absolute left-[5px] z-30 h-[50px] w-[50px] rounded-[25px] bg-blue-500 opacity-50'}
		></div>
	{/if}
{/snippet}

<div class="flex h-full w-full gap-2 border">
	{#each Object.keys(handles) as key}
		<div class="flex flex-1 flex-col items-center justify-center gap-2 p-2">
			<p>{key} {handles[key].value}</p>
			<div
				bind:this={handles[key].parent}
				class=" relative z-20 h-[60%] w-[60px] rounded-[30px] bg-red-500 opacity-50"
			>
				{@render thumb(false, handles[key].value)}
			</div>
		</div>
	{/each}
</div>
