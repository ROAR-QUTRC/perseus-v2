<script lang="ts" module>
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Resource Monitor';
	export const description =
		'Shows the CPU, memory, and network usage of the device that Perseus-UI is currently running on';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			general: {
				dataPointsToStore: {
					type: 'number',
					description: 'The number of data points to store in each chart',
					value: '25'
				}
			},
			memory: {
				sizeFormat: {
					type: 'select',
					description: 'Format to display memory size',
					options: [
						{ value: 'bytes', label: 'bytes' },
						{ value: 'KB', label: 'KB' },
						{ value: 'MB', label: 'MB' },
						{ value: 'GB', label: 'GB' }
					],
					value: 'GB'
				},
				divideBy1024: {
					type: 'switch',
					description: 'Calculate memory using 1024 not 1000'
				}
			},
			network: {
				networkInterface: {
					type: 'select',
					description: 'Network interface to monitor',
					options: []
				},
				setUsageScale: {
					type: 'number',
					description: 'Set the maximum value on the y axis in KB/s',
					value: '5000'
				}
			}
		}
	});
</script>

<script lang="ts">
	import { io, Socket } from 'socket.io-client';
	import { onMount } from 'svelte';
	import Chart, { type ChartConfiguration } from 'chart.js/auto';

	interface MonitorData {
		cpu: number;
		memory: {
			total: {
				total: number;
				used: number;
			};
			normal: {
				total: number;
				used: number;
			};
			swap: {
				total: number;
				used: number;
			};
		};
		network: Record<string, { rxBytes: number; txBytes: number }>;
	}

	const socket: Socket = io();

	let cpuCanvas = $state<HTMLCanvasElement>();
	let cpuChart: Chart;
	let memoryCanvas = $state<HTMLCanvasElement>();
	let memoryChart: Chart;
	let networkCanvas = $state<HTMLCanvasElement>();
	let networkChart: Chart;

	socket.on('monitor-data', (data: MonitorData) => {
		pushChartData(cpuChart, ['CPU Usage'], [data.cpu]);
		cpuChart.update();

		memoryChart.options.scales!.y!.max = convertBytes(
			data.memory.total.total,
			settings.groups.memory.sizeFormat.value!
		);
		pushChartData(
			memoryChart,
			['Total', 'Memory', 'Swap'],
			[
				convertBytes(data.memory.total.used, settings.groups.memory.sizeFormat.value!),
				convertBytes(data.memory.normal.used, settings.groups.memory.sizeFormat.value!),
				convertBytes(data.memory.swap.used, settings.groups.memory.sizeFormat.value!)
			]
		);
		memoryChart.update();

		const maxValue = Number(settings.groups.network.setUsageScale.value);
		networkChart.options.scales!.y!.max = maxValue;
		// make sure an interface is selected
		if (settings.groups.network.networkInterface.value === undefined) {
			settings.groups.network.networkInterface.options = Object.keys(data.network).map((key) => ({
				value: key,
				label: key
			}));
			settings.groups.network.networkInterface.value =
				settings.groups.network.networkInterface.options[0].value;
		}
		pushChartData(
			networkChart,
			['Upload', 'Download', '80% marker'],
			[
				convertBytes(data.network[settings.groups.network.networkInterface.value].txBytes, 'KB'),
				convertBytes(data.network[settings.groups.network.networkInterface.value].rxBytes, 'KB'),
				maxValue * 0.8
			]
		);
		networkChart.update();
	});

	onMount(() => {
		cpuChart = new Chart(
			cpuCanvas?.getContext('2d')!,
			getChartConfig('CPU Usage (%)', [{ label: 'CPU Usage', data: [] }], 100)
		);
		memoryChart = new Chart(
			memoryCanvas?.getContext('2d')!,
			getChartConfig('Memory Usage', [
				{ label: 'Total', data: [] },
				{ label: 'Memory', data: [] },
				{ label: 'Swap', data: [] }
			])
		);
		networkChart = new Chart(
			networkCanvas?.getContext('2d')!,
			getChartConfig('Network Usage (KB/s)', [
				{ label: 'Upload', data: [] },
				{ label: 'Download', data: [] },
				{
					label: '80% marker',
					data: [],
					pointStyle: false
				}
			])
		);

		return () => {
			socket.disconnect();
		};
	});

	const pushChartData = (chart: Chart, datasets: string[], values: number[]) => {
		const limit = Number(settings.groups.general.dataPointsToStore.value);
		if (chart.data.labels?.length! < limit) chart.data.labels?.push('');
		else chart.data.labels = chart.data.labels?.slice(-limit);

		for (let i = 0; i < chart.data.datasets.length; i++) {
			if (chart.data.datasets[i].label === datasets[i]) {
				chart.data.datasets[i].data.push(values[i]);

				if (chart.data.datasets[i].data.length > limit) {
					chart.data.datasets[i].data = chart.data.datasets[i].data.slice(-limit);
				}
			}
		}
	};

	const getChartConfig = (
		yAxisTitle: string,
		datasets: { label: string; data: number[]; pointStyle?: boolean }[],
		yMax?: number
	): ChartConfiguration => {
		return {
			data: {
				labels: [],
				datasets: datasets
			},
			type: 'line',
			options: {
				scales: {
					y: {
						beginAtZero: true,
						max: yMax,
						title: {
							display: true,
							text: yAxisTitle
						}
					}
				},
				animation: false
			}
		};
	};

	$effect(() => {
		const maxValue = Number(settings.groups.network.setUsageScale.value);
		if (networkChart !== undefined) {
			networkChart.options.scales!.y!.max = maxValue;
			networkChart.data.datasets[2].data = Array(
				Number(settings.groups.general.dataPointsToStore.value)
			).fill(maxValue * 0.8);
			networkChart.update();
		}
	});

	const convertBytes = (bytes: number, format: string) => {
		const sizes = ['bytes', 'KB', 'MB', 'GB'];
		const i = sizes.indexOf(format);
		if (i === 0) return bytes;
		if (settings.groups.memory.divideBy1024.value) return bytes / Math.pow(1024, i);
		return bytes / Math.pow(1000, i);
	};
</script>

<div class="flex h-full w-full flex-wrap">
	<div class="">
		<canvas bind:this={cpuCanvas}></canvas>
	</div>
	<div class="">
		<canvas bind:this={memoryCanvas}></canvas>
	</div>
	<div class="">
		<canvas bind:this={networkCanvas}></canvas>
	</div>
</div>
