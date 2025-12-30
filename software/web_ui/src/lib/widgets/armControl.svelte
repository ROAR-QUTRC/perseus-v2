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
				automaticallySendMessages: {
					description: 'Disables automatic sending of control messages',
					type: 'switch',
					value: 'false'
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
	import { onMount, untrack } from 'svelte';
	import { Button } from '$lib/components/ui/button/index.js';
	import { Minus, Plus } from 'svelte-radix';
	import ScrollArea from '$lib/components/ui/scroll-area/scroll-area.svelte';
	import * as changeCase from 'change-case';
	import Input from '$lib/components/ui/input/input.svelte';

	let topic: ROSLIB.Topic<{ data: Array<number> }> | null = null;
	let automaticPositionMessages = $derived(settings.groups.general.automaticallySendMessages.value === 'true');
	$effect(() => {
		if (!automaticPositionMessages && interval) {
			clearInterval(interval);
			console.log('Cleared interval for automatic position messages');
			interval = null;
		}
		untrack(() => {
			settings.groups.general.messageFrequency.disabled = !automaticPositionMessages;
			if (automaticPositionMessages) {
				startTimeout();
			}
		});
	});

	// String number record type for indexing in markdown
	let positions = $state<Record<string, number>>({
		elbow: 0,
		wristPan: 0,
		wristTilt: 0,
		shoulderPan: 0,
		shoulderTilt: 0,
	});

	const gearReductions: Record<string, number> = {
		wristPan: 19,
		wristTilt: 1,
		shoulderPan: 1,
		shoulderTilt: 1,
		elbow: 1,
	};

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

	let interval: NodeJS.Timeout | null = null;

	const sendPositions = () => {
		if (topic) {
			const message = {
				// Index of this array corresponds to motor ID - 1
				data: [
					positions.elbow * gearReductions.elbow,
					positions.wristPan * gearReductions.wristPan,
					positions.wristTilt * gearReductions.wristTilt
				]
			};
			console.log('Publishing arm positions:', message.data);
			topic.publish(message);
		}
	}

	const startTimeout = () => {
		if (automaticPositionMessages) {
			const frequency = Number(settings.groups.general.messageFrequency.value);
			interval = setInterval(() => {
				sendPositions();
			}, 1000 / frequency);
		}
	}

	onMount(() => {
		return () => {
			if (interval) {
				clearInterval(interval);
				interval = null;
			}
		};
	});

	// editing positions directly
	// let editIndex = $state<number | null>(null);
	// let editValue = $state<number>(0);
	// let inputElement = $state();

	// const startEdit = (index: number) => {
	// 	editIndex = index;
	// 	const joint = Object.keys(positions)[index];
	// 	editValue = positions[joint];
	// };

</script>

<ScrollArea class="h-full w-full pr-2">
	<Button class="m-2" onclick={sendPositions} variant="outline" disabled={automaticPositionMessages}>Send Positions</Button>
	<div class="flex">
		{#each Object.keys(positions) as joint, index}
			<div class="flex flex-col flex-1">
				<p class="text-center">{changeCase.sentenceCase(joint)}:</p>
				<div class="py-2 flex flex-col items-center gap-2 border w-1/2 mx-auto rounded-xl">
					<button onclick={() => (positions[joint] += increment)}>
						<Plus />
					</button>
					<!-- {#if editIndex === index}
						<Input class="border-0 border-t border-b w-full text-center" bind:value={editValue} bind:this={inputElement} />
					{:else} -->
						<button class="border-0 border-t border-b w-full text-center py-2">{positions[joint]}&deg;</button>
					<!-- {/if} -->
					<button onclick={() => (positions[joint] -= increment)}>
						<Minus />
					</button>
				</div>
				<p class="opacity-40 text-center">Output: 1:{gearReductions[joint]}</p>
			</div>
		{/each}
	</div>
</ScrollArea>
