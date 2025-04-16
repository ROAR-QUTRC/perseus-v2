<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Bus Power Manager';
	export const description = 'Enable and disable different buses on the rover control board';
	export const group = 'CAN Bus';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	import { ros } from '$lib/scripts/ros.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import ROSLIB from 'roslib';
	import Fa from 'svelte-fa';
	import { faPowerOff } from '@fortawesome/free-solid-svg-icons';
	import { onMount } from 'svelte';
	import { sentenceCase } from 'change-case';

	// Widget logic goes here
	let busState = $state<Record<string, { status: string; current: string; voltage: string }>>({});

	const toggleBusPower = (e: Event, bus: string) => {
		e.preventDefault();
		const publisher = new ROSLIB.Topic({
			ros: ros.value!,
			name: '/ros_to_can',
			messageType: 'std_msgs/String'
		});

		const message = new ROSLIB.Message({
			data: JSON.stringify({ bus: bus, on: busState[bus].status ? '1' : '0' })
		});

		publisher.publish(message);
		publisher.unsubscribe();
	};

	onMount(() => {
		const listener = new ROSLIB.Topic({
			ros: ros.value!,
			name: '/can_to_ros',
			messageType: 'std_msgs/String'
		});

		listener.subscribe((message: any) => {
			busState = JSON.parse(message.data);
		});

		return () => {
			listener.unsubscribe();
		};
	});
</script>

<div class="flex h-full w-full">
	{#each Object.keys(busState) as bus, i}
		<div class="mx-[5px] mb-auto w-[calc(25%-10px)] min-w-[60px] max-w-[130px] flex-col">
			<p class="mb-2 text-center">{sentenceCase(bus)}</p>
			<button
				class="aspect-square cursor-pointer rounded-[50%] border"
				onclick={(e) => toggleBusPower(e, bus)}
			>
				<Fa
					icon={faPowerOff}
					class="power-button"
					color={busState[bus].status == '1' ? '#f00' : '#0f0'}
				/>
			</button>
			<p class="mt-2">Current: {busState[bus].current}</p>
			<p>Voltage: {busState[bus].voltage}</p>
		</div>
	{:else}
		<p>Looking for buses... Make sure the rosbridge is connected.</p>
	{/each}
</div>

<style>
	:global(.power-button) {
		width: 80%;
		height: 100%;
		margin: auto;
	}
</style>
