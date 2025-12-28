<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Arm Controller';
	// These properties are optional
	export const description = 'Controller for each of the post landing arm servos';
	export const group: WidgetGroupType = 'ROS';
	export const isRosDependent = true; // Set to true if the widget requires a ROS connection

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			general: {
				incrementValue: {
					type: 'number',
					value: '5'
				},
				messageFrequency: {
					description: 'Frequency (Hz) to send control messages at',
					type: 'number',
					value: '10'
				}
			}
		}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import * as ROSLIB from 'roslib';
	import { onMount } from 'svelte';
	import { Button } from '$lib/components/ui/button/index.js';
	import { Slider } from '$lib/components/ui/slider/index.js';
	import { Minus, Plus } from 'svelte-radix';
	import ScrollArea from '$lib/components/ui/scroll-area/scroll-area.svelte';
	import * as changeCase from 'change-case';

	let topic: ROSLIB.Topic<{ data: Array<number> }> | null = null;

	// String number record type for indexing in markdown
	let positions = $state<Record<string, number>>({
		shoulderPan: 0,
		shoulderTilt: 0,
		elbow: 0,
		wristPan: 0,
		wristTilt: 0
	});

	let increment = $derived(Number(settings.groups.general.incrementValue.value));

	$effect(() => {
		const ros = getRosConnection();
		if (ros) {
			topic = new ROSLIB.Topic({
				ros: ros,
				name: '/arm/rmd_control',
				messageType: 'std_msgs/msg/Float64MultiArray'
			});
		} else {
			topic = null;
		}
	});

	onMount(() => {
		const frequency = Number(settings.groups.general.messageFrequency.value);
		const interval = setInterval(() => {
			if (topic) {
				const message = {
					data: [
						positions.shoulderPan,
						positions.shoulderTilt,
						positions.elbow,
						positions.wristPan,
						positions.wristTilt
					]
				};
				topic.publish(message);
			}
		}, 1000 / frequency);

		return () => {
			clearInterval(interval);
		};
	});
</script>

<ScrollArea class="flex h-full w-full flex-col pr-2">
	{#each Object.keys(positions) as joint}
		<div class="">
			<p class="text-center">{changeCase.sentenceCase(joint)}: {positions[joint]}</p>
			<div class="flex flex-row items-center gap-2">
				<Button onclick={() => (positions[joint] -= increment)} size="icon" variant="ghost"
					><Minus /></Button
				>
				<Slider type="single" bind:value={positions[joint]} max={360} min={0} />
				<Button onclick={() => (positions[joint] += increment)} size="icon" variant="ghost"
					><Plus /></Button
				>
			</div>
		</div>
	{/each}
</ScrollArea>
