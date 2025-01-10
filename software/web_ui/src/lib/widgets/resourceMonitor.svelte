<script lang="ts" module>
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';
	import { onMount } from 'svelte';

	export const name = 'Resource Monitor';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			settings: {
				refreshRate: {
					type: 'number',
					description: 'Refresh rate in milliseconds',
					value: '1000'
				},
				dataPointsToStore: {
					type: 'number',
					description: 'The number of data points to store in each chart',
					value: '50'
				},
				networkInterface: {
					type: 'select',
					description: 'Network interface to monitor',
					options: []
				},
				reloadNetworkInterfaces: {
					type: 'button',
					action: () => {
						// updated network interfaces
						return 'Not implemented';
					}
				}
			}
		}
	});
</script>

<script lang="ts">
	import Button from '$lib/components/ui/button/button.svelte';
	import Chart from 'chart.js/auto';

	// validate the refreate settings
	$effect(() => {
		if (Number(settings.groups.settings.refreshRate.value) < 500) {
			settings.groups.settings.refreshRate.value = '500';
		}
	});
	let cpuCanvas = $state<HTMLCanvasElement>();
	let cpuChart: Chart;
	let cpuTimeoutId: NodeJS.Timeout;
	let memCanvas = $state<HTMLCanvasElement>();
	let netCanvas = $state<HTMLCanvasElement>();

	const updateData = () => {
		fetch('/api/widget/resources', { method: 'GET' }).then((res) => {
			if (res.ok) {
				res.json().then((d) => {
					while (
						cpuChart.data.datasets[0].data.length >
						Number(settings.groups.settings.dataPointsToStore.value)
					) {
						cpuChart.data.datasets[0].data.shift();
						cpuChart.data.labels?.shift();
					}
					cpuChart.data.datasets[0].data.push(d.cpuUsage);
					cpuChart.data.labels?.push(new Date().toLocaleTimeString('en-GB'));
					cpuChart.update();
				});
			}
		});

		cpuTimeoutId = setTimeout(() => {
			updateData();
		}, Number(settings.groups.settings.refreshRate.value));
	};

	onMount(() => {
		cpuChart = new Chart(cpuCanvas?.getContext('2d')!, {
			data: {
				labels: [],
				datasets: [{ label: 'CPU usage', data: [] }]
			},
			type: 'line',
			options: {
				scales: {
					y: {
						beginAtZero: true,
						max: 100,
						title: {
							display: true,
							text: 'CPU Usage (%)'
						}
					}
				},
				animation: false
			}
		});

		// start intervals
		updateData();

		return () => {
			cpuChart.destroy();
			clearTimeout(cpuTimeoutId);
		};
	});
</script>

<!-- <Button onclick={updateData}>New Data</Button> -->
<canvas bind:this={cpuCanvas}></canvas>
<!-- <div class="flex h-full flex-wrap">
	<div class="-ml-[1px] -mt-[1px] basis-1/4 border p-1">
		<strong>CPU</strong>
	</div>
	<div class="-ml-[1px] -mt-[1px] basis-1/4 border p-1">
		<strong>Memory</strong>
		<canvas bind:this={memCanvas}></canvas>
	</div>
	<div class="-ml-[1px] -mt-[1px] basis-1/2 border p-1">
		<strong>Network</strong>
		<canvas bind:this={netCanvas}></canvas>
	</div>
</div> -->
