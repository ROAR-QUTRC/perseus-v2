<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Bus Power Manager';
	export const description = 'Enable and disable different buses on the rover control board';
	export const group = 'CAN Bus';
	export const isRosDependent = true;

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/ros-bridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import ROSLIB from 'roslib';
	import Fa from 'svelte-fa';
	import { faPowerOff } from '@fortawesome/free-solid-svg-icons';
	import { onMount } from 'svelte';
	import { sentenceCase } from 'change-case';
	import * as Dialog from '$lib/components/ui/dialog/index';
	import { Button } from '$lib/components/ui/button';

	const busStatus = [
		'OFF',
		'ON',
		'PRECHARGING',
		'SHORT_CIRCUIT',
		'SWITCH_FAILED',
		'OVERLOAD',
		'FAULT'
	];

	let listener: ROSLIB.Topic;
	let busState = $state<Record<string, { status: number; current: string; voltage: string }>>({});

	$inspect(busState);

	const toggleBusPower = (e: Event, bus: string) => {
		e.preventDefault();
		const publisher = new ROSLIB.Topic({
			ros: getRosConnection() as ROSLIB.Ros,
			name: '/ros_to_can',
			messageType: 'std_msgs/String'
		});

		const message = new ROSLIB.Message({
			data: JSON.stringify({ bus: bus, on: busState[bus].status !== 1 ? '1' : '0' })
		});

		publisher.publish(message);
		publisher.unsubscribe();
	};

	$effect(() => {
		if (getRosConnection()) {
			listener = new ROSLIB.Topic({
				ros: getRosConnection() as ROSLIB.Ros,
				name: '/can_to_ros',
				messageType: 'std_msgs/String'
			});

			listener.subscribe((message: any) => {
				const { name, status, ...busData } = JSON.parse(message.data);
				busState[name] = { status: Number(status), ...busData };
			});
		}
	});

	onMount(() => {
		return () => {
			listener.unsubscribe();
		};
	});

	// Confirm menu
	let openConfirm: Record<string, boolean> = {};
	// $effect(() => {
	// 	for (const bus of Object.keys(busState)) {
	// 		if (!(bus in openConfirm)) {
	// 			openConfirm[bus] = false;
	// 		}
	// 	}
	// });
</script>

<div class="flex w-full">
	{#each Object.keys(busState) as bus, i}
		<div
			class="mx-[5px] mb-auto flex w-[calc(25%-10px)] min-w-[60px] max-w-[130px] flex-col justify-center rounded-lg border p-2"
		>
			<p class="mb-2 text-center">{sentenceCase(bus)}</p>

			<Dialog.Root bind:open={openConfirm[bus]}>
				<Dialog.Trigger>
					<button
						class="aspect-square cursor-pointer rounded-[50%] border"
						onclick={() => (openConfirm[bus] = true)}
					>
						<Fa
							icon={faPowerOff}
							class="power-button w-full"
							color={busState[bus].status == 1 ? '#0f0' : '#f00'}
						/>
					</button>
				</Dialog.Trigger>
				<Dialog.Content>
					<Button>Cancel</Button>
					<Button>Confirm</Button>
				</Dialog.Content>
			</Dialog.Root>

			<p>{busStatus[busState[bus].status]}</p>
			<p class="mt-2">Current: {busState[bus].current}</p>
			<p>Voltage: {busState[bus].voltage}</p>
		</div>
	{:else}
		<p>Looking for buses... Make sure the rosbridge is connected.</p>
	{/each}
</div>
<p>Note: <kbd>FAULT</kbd> on the drive bus likely indicates the drive stop is engaged.</p>

<style>
	:global(.power-button) {
		width: 80%;
		height: 100%;
		margin: auto;
	}
</style>
