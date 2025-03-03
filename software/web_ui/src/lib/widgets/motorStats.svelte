<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Motor Stats';
	export const description = 'View the current ';
	export const group = 'CAN Bus';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			General: {
				PrimaryReadout: {
					type: 'select',
					description: 'The primary readout for the widget',
					options: [
						{ value: 'rpm', label: 'RPM' },
						{ value: 'difference', label: 'Difference' },
						{ value: 'amps', label: 'Amps' },
						{ value: 'temperature', label: 'Temperature' }
					],
					value: 'rpm'
				},
				SecondaryReadout: {
					type: 'select',
					description: 'The secondary readout for the widget',
					options: [
						{ value: 'rpm', label: 'RPM' },
						{ value: 'difference', label: 'Difference' },
						{ value: 'amps', label: 'Amps' },
						{ value: 'temperature', label: 'Temperature' }
					],
					value: 'temperature'
				}
			}
		}
	});
</script>

<script lang="ts">
	import Button from '$lib/components/ui/button/button.svelte';
	import Gauge from './motorStats/gauge.svelte';
	import { onMount } from 'svelte';
	// import { ros } from '$lib/scripts/ros.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	// import ROSLIB from 'roslib';

	// Widget logic goes here

	let data = $state([
		{ targetRPM: 0, currentRPM: 0, currentAmps: 0, currentTemp: 0 },
		{ targetRPM: 0, currentRPM: 0, currentAmps: 0, currentTemp: 0 },
		{ targetRPM: 0, currentRPM: 0, currentAmps: 0, currentTemp: 0 }
	]);

	//set random data every second
	const update = () => {
		data[0] = {
			targetRPM: Math.random() * 200 - 100,
			currentRPM: Math.random() * 200 - 100,
			currentAmps: Math.random() * 200 - 100,
			currentTemp: Math.random() * 200 - 100
		};
		data[1] = {
			targetRPM: Math.random() * 200 - 100,
			currentRPM: Math.random() * 200 - 100,
			currentAmps: Math.random() * 200 - 100,
			currentTemp: Math.random() * 200 - 100
		};
		data[2] = {
			targetRPM: Math.random() * 200 - 100,
			currentRPM: Math.random() * 200 - 100,
			currentAmps: Math.random() * 200 - 100,
			currentTemp: Math.random() * 200 - 100
		};
	};

	onMount(async () => {
		// setInterval(update, 1000);
	});
</script>

<div class="flex flex-row flex-wrap">
	{#each data as motor}
		<Gauge
			rpm={motor.currentRPM}
			targetRpm={motor.targetRPM}
			current={motor.currentAmps}
			temperature={motor.currentTemp}
		/>
	{/each}
</div>
<div class="m-2 flex w-fit gap-2 rounded-lg border p-2">
	<p class="m-auto">Legend:</p>
	<p class="legend target">Target RPM</p>
	<p class="legend rpm">RPM</p>
	<p class="legend current">Current</p>
</div>
<Button onclick={update}>Update Data</Button>

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
