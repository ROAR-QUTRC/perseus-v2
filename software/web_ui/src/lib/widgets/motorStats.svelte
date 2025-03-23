<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Motor Stats - Under development';
	export const description = 'View live stats for each of the motors.';
	export const group = 'CAN Bus';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			General: {
				PrimaryReadout: {
					type: 'select',
					description: 'The primary readout for the widget',
					options: [
						{ value: 'rpm', label: 'RPM' },
						{ value: 'difference', label: 'Target and current RPM diff' },
						{ value: 'current', label: 'Amps' },
						{ value: 'temperature', label: 'Temperature' }
					],
					value: 'rpm'
				},
				SecondaryReadout: {
					type: 'select',
					description: 'The secondary readout for the widget',
					options: [
						{ value: 'rpm', label: 'RPM' },
						{ value: 'difference', label: 'Target and current RPM difference' },
						{ value: 'current', label: 'Amps' },
						{ value: 'temperature', label: 'Temperature' }
					],
					value: 'temperature'
				}
			}
		}
	});

	export interface MotorStatsType {
		targetRPM: number;
		currentRPM: number;
		current: number;
		temperature: number;
	}
</script>

<script lang="ts">
	// Switch this to twist stamped messages
	// publish twist stamp messages for joystick control
	import Button from '$lib/components/ui/button/button.svelte';
	import Gauge from './motorStats/gauge.svelte';
	import { onMount } from 'svelte';
	// import { ros } from '$lib/scripts/ros.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	// import ROSLIB from 'roslib';

	// Widget logic goes here

	let data = $state<MotorStatsType[]>([
		{ targetRPM: 0, currentRPM: 0, current: 0, temperature: 0 },
		{ targetRPM: 0, currentRPM: 0, current: 0, temperature: 0 },
		{ targetRPM: 0, currentRPM: 0, current: 0, temperature: 0 },
		{ targetRPM: 0, currentRPM: 0, current: 0, temperature: 0 }
	]);

	//set random data every second
	const update = () => {
		data[0] = {
			targetRPM: Math.random() * 200 - 100,
			currentRPM: Math.random() * 200 - 100,
			current: Math.random() * 200 - 100,
			temperature: Math.random() * 200 - 100
		};
		data[1] = {
			targetRPM: Math.random() * 200 - 100,
			currentRPM: Math.random() * 200 - 100,
			current: Math.random() * 200 - 100,
			temperature: Math.random() * 200 - 100
		};
		data[2] = {
			targetRPM: Math.random() * 200 - 100,
			currentRPM: Math.random() * 200 - 100,
			current: Math.random() * 200 - 100,
			temperature: Math.random() * 200 - 100
		};
		data[3] = {
			targetRPM: Math.random() * 200 - 100,
			currentRPM: Math.random() * 200 - 100,
			current: Math.random() * 200 - 100,
			temperature: Math.random() * 200 - 100
		};
	};

	onMount(async () => {
		setInterval(update, 1000);
	});
</script>

<div class="flex flex-row flex-wrap">
	{#each data as motor}
		<Gauge
			motorData={motor}
			primaryReadoutType={settings.groups.General.PrimaryReadout.value}
			secondaryReadoutType={settings.groups.General.SecondaryReadout.value}
		/>
	{/each}
</div>
<div class="m-2 flex w-fit gap-2 rounded-lg border p-2">
	<p class="m-auto">Legend:</p>
	<p class="legend target">Target RPM</p>
	<p class="legend rpm">RPM</p>
	<p class="legend current">Current</p>
</div>

<!-- <Button onclick={update}>Update Data</Button> -->

<style>
	.legend {
		border-radius: 20px;
		padding: 2px 10px;
	}

	.target {
		background-color: #db26293f;
	}

	.rpm {
		background-color: #db2629;
	}

	.current {
		background-color: #00f;
		color: white;
	}
</style>
