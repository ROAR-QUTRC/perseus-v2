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
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import * as ROSLIB from 'roslib';
	import Fa from 'svelte-fa';
	import { faPowerOff } from '@fortawesome/free-solid-svg-icons';
	import { onMount } from 'svelte';
	import { sentenceCase } from 'change-case';
	import * as AlertDialog from '$lib/components/ui/alert-dialog/index.js';

	const busStatus = [
		'OFF',
		'ON',
		'PRECHARGING',
		'SHORT_CIRCUIT',
		'SWITCH_FAILED',
		'OVERLOAD',
		'FAULT'
	];

	let listener: ROSLIB.Topic | null = null;
	let busState = $state<
		Record<string, { status: number; current: number; voltage: number; openAlert: boolean }>
	>({
		compute: { status: 0, current: 0, voltage: 0, openAlert: false },
		drive: { status: 0, current: 0, voltage: 0, openAlert: false },
		aux: { status: 0, current: 0, voltage: 0, openAlert: false },
		spare: { status: 0, current: 0, voltage: 0, openAlert: false }
	});

	const toggleBusPower = (e: Event, bus: string) => {
		e.preventDefault();

		// close the dialog
		busState[bus].openAlert = false;

		const publisher = new ROSLIB.Topic({
			ros: getRosConnection() as ROSLIB.Ros,
			name: '/ros_to_can',
			messageType: 'std_msgs/String'
		});

		const message = new ROSLIB.Message({
			data: JSON.stringify({
				bus: bus,
				on: busState[bus].status !== 1 ? '1' : '0',
				clear: busState[bus].status === 6 || busState[bus].status === 4 ? '1' : '0' // Clear FAULT or SWITCH_FAILED
			})
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
				const { name, status, current, voltage } = JSON.parse(message.data);
				busState[name] = {
					status: Number(status),
					current: Number(current),
					voltage: Number(voltage),
					openAlert: busState[name]?.openAlert ?? false
				};
			});
		}
	});

	onMount(() => {
		return () => {
			listener?.unsubscribe();
		};
	});
</script>

<div class="flex w-full flex-wrap">
	{#each Object.keys(busState) as bus}
		<div class="m-2 flex min-w-[145px] flex-col justify-center rounded-lg border p-2">
			<p class="mb-2 text-center">{sentenceCase(bus)}: {busStatus[busState[bus].status]}</p>

			<AlertDialog.Root bind:open={busState[bus].openAlert}>
				<AlertDialog.Trigger>
					<button class="aspect-square cursor-pointer rounded-[50%] border">
						<Fa
							icon={faPowerOff}
							class="power-button h-[50px] w-[50px]"
							color={busState[bus].status !== 1 ? 'red' : 'green'}
							scale={3}
						/>
					</button>
				</AlertDialog.Trigger>
				<AlertDialog.Content>
					<AlertDialog.Header>
						<AlertDialog.Title>
							Power {busState[bus].status !== 1 ? 'On' : 'Off'}
							{sentenceCase(bus)}?
						</AlertDialog.Title>
					</AlertDialog.Header>
					<AlertDialog.Footer>
						<AlertDialog.Cancel>Cancel</AlertDialog.Cancel>
						<AlertDialog.Action onclick={(e) => toggleBusPower(e, bus)}>Confirm</AlertDialog.Action>
					</AlertDialog.Footer>
				</AlertDialog.Content>
			</AlertDialog.Root>
			<p class="mt-2 text-center">Current: {(Number(busState[bus].current) / 1000).toFixed(3)}A</p>
			<p class="text-center">Voltage: {(Number(busState[bus].voltage) / 1000).toFixed(3)}V</p>
		</div>
	{:else}
		<p>Waiting for power bus data</p>
	{/each}
</div>
<p class="ml-2 opacity-35">FAULT on the drive bus likely indicates the drive stop is engaged.</p>

<style>
	:global(.power-button) {
		width: 80%;
		height: 100%;
		margin: auto;
	}
</style>
