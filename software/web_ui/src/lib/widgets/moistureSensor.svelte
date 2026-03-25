<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Kibisis Moisture Sensor';
	// These properties are optional
	export const description = 'Trigger a moisture sample and view ambient vs illuminated LDR readings.';
	export const group: WidgetGroupType = 'ROS';
	export const isRosDependent = true; // Set to true if the widget requires a ROS connection

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			topics: {
				sampleService: {
					type: 'text',
					description: 'ROS2 service name to trigger a sample',
					value: '/kibisis/moisture/sample'
				},
				valueTopic: {
					type: 'text',
					description: 'Topic publishing the moisture ADC result',
					value: '/kibisis/moisture/value'
				}
			}
		}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import * as ROSLIB from 'roslib';
	import { Button } from '$lib/components/ui/button/index.js';
	import { onMount } from 'svelte';

	// ROS handles
	let sampleService = $state<ROSLIB.Service | null>(null);
	let valueSub: ROSLIB.Topic | null = null;

	// UI state
	let moistureValue = $state<number | null>(null);
	let isSampling = $state(false);
	let statusMessage = $state('ROS disconnected');

	// ROS2 connection management
	$effect(() => {
		const ros = getRosConnection();

		if (!ros) {
			sampleService = null;
			valueSub?.unsubscribe();
			valueSub = null;
			statusMessage = 'ROS disconnected';
			return;
		}

		sampleService = new ROSLIB.Service({
			ros,
			name: settings.groups.topics.sampleService.value ?? '/kibisis/moisture/sample',
			serviceType: 'std_srvs/Trigger'
		});

		valueSub = new ROSLIB.Topic({
			ros,
			name: settings.groups.topics.valueTopic.value ?? '/kibisis/moisture/value',
			messageType: 'std_msgs/Float32'
		});

		valueSub.subscribe((msg: any) => {
			moistureValue = msg.data;
			isSampling = false;
			statusMessage = 'Sample received';
		});

		statusMessage = 'ROS connected';

		return () => {
			valueSub?.unsubscribe();
			valueSub = null;
		};
	});

	const triggerSample = () => {
		if (!sampleService) {
			statusMessage = 'Service not ready';
			return;
		}

		isSampling = true;
		statusMessage = 'Sampling…';

		sampleService.callService(
			new ROSLIB.ServiceRequest({}),
			(resp: any) => {
				if (!resp?.success) {
					isSampling = false;
					statusMessage = resp?.message ?? 'Sample failed';
				} else {
					statusMessage = resp?.message ?? 'Waiting for result…';
				}
			},
			(err: any) => {
				isSampling = false;
				statusMessage = `Error: ${err?.toString?.() ?? err}`;
			}
		);
	};

	// Display helper — map 0-4095 to a percentage
	const toPercent = (v: number) => ((v / 4095) * 100).toFixed(1);
</script>

<div class="flex h-full w-full flex-col gap-4 p-3">
	<!-- Reading display -->
	<div class="flex flex-1 flex-col items-center justify-center gap-2">
		{#if moistureValue !== null}
			<p class="text-5xl font-bold tabular-nums">
				{toPercent(moistureValue)}<span class="text-2xl opacity-60">%</span>
			</p>
			<p class="text-xs opacity-50">Raw ADC: {moistureValue.toFixed(0)} / 4095</p>
		{:else}
			<p class="text-xl opacity-40">No reading yet</p>
		{/if}
	</div>

	<!-- Sample button -->
	<Button
		class="w-full"
		disabled={isSampling || !sampleService}
		onclick={triggerSample}
	>
		{isSampling ? 'Sampling…' : 'Take Sample'}
	</Button>

	<!-- Status -->
	<p class="text-xs opacity-60">Status: <span class="font-mono">{statusMessage}</span></p>
</div>