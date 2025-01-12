<script lang="ts" module>
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';
	import { onMount } from 'svelte';

	export const name = 'Resource Monitor';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			general: {
				refreshRate: {
					type: 'number',
					description: 'Refresh rate in milliseconds',
					value: '1000'
				},
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
	import Chart from 'chart.js/auto';

	// validate the refreate settings
	$effect(() => {
		if (Number(settings.groups.general.refreshRate.value) < 500) {
			settings.groups.general.refreshRate.value = '500';
		}
	});

	// update the y axis of the network chart
	$effect(() => {
		const maxValue = Number(settings.groups.network.setUsageScale.value);
		if (netChart !== undefined) {
			netChart.options.scales!.y!.max = maxValue;
			netChart.data.datasets[2].data = Array(
				Number(settings.groups.general.dataPointsToStore.value)
			).fill(maxValue * 0.8);
			netChart.update();
		}
	});

	const convertBytes = (bytes: number, format: string) => {
		const sizes = ['bytes', 'KB', 'MB', 'GB'];
		const i = sizes.indexOf(format);
		if (i === 0) return bytes;
		if (settings.groups.memory.divideBy1024.value) return bytes / Math.pow(1024, i);
		return bytes / Math.pow(1000, i);
	};

	// CPU
	let cpuCanvas = $state<HTMLCanvasElement>();
	let cpuChart: Chart;

	// Memory
	let memCanvas = $state<HTMLCanvasElement>();
	let memChart: Chart;

	// Network
	let netCanvas = $state<HTMLCanvasElement>();
	let netChart: Chart;

	let timeoutId: NodeJS.Timeout;
	const updateData = () => {
		// limit the length of the data
		const limit = Number(settings.groups.general.dataPointsToStore.value);
		cpuChart.data.datasets[0].data = cpuChart.data.datasets[0].data.slice(-limit);
		cpuChart.data.labels = cpuChart.data.labels?.slice(-limit);

		memChart.data.labels = memChart.data.labels?.slice(-limit);
		for (let i = 0; i < memChart.data.datasets.length; i++) {
			memChart.data.datasets[i].data = memChart.data.datasets[i].data.slice(-limit);
		}

		netChart.data.labels = netChart.data.labels?.slice(-limit);
		for (let i = 0; i < 2; i++) {
			netChart.data.datasets[i].data = netChart.data.datasets[i].data.slice(-limit);
		}

		// fetch data
		fetch('/api/widget/resources', {
			method: 'POST',
			headers: {
				'Content-Type': 'application/json'
			},
			body: JSON.stringify({ networkIf: settings.groups.network.networkInterface.value })
		}).then((res) => {
			if (res.ok) {
				res.json().then((d) => {
					// CPU
					cpuChart.data.datasets[0].data.push(JSON.parse(d[0]).cpuUsage);
					cpuChart.data.labels?.push('');
					cpuChart.update();

					// Memory
					memChart.options.scales!.y!.max = convertBytes(
						Number(JSON.parse(d[1]).total[0]),
						settings.groups.memory.sizeFormat.value!
					);
					memChart.data.labels?.push('');

					memChart.data.datasets[0].data.push(
						convertBytes(
							Number(JSON.parse(d[1]).total[1]),
							settings.groups.memory.sizeFormat.value!
						) // total
					);
					memChart.data.datasets[1].data.push(
						convertBytes(
							Number(JSON.parse(d[1]).total[1]) - Number(JSON.parse(d[1]).swap[1]),
							settings.groups.memory.sizeFormat.value!
						) // memory
					);
					memChart.data.datasets[2].data.push(
						convertBytes(Number(JSON.parse(d[1]).swap[1]), settings.groups.memory.sizeFormat.value!) // swap
					);
					memChart.update();

					// Network
					netChart.data.labels?.push('');
					netChart.data.datasets[0].data.push(
						convertBytes(
							Number(JSON.parse(d[2]).tx) *
								(1 / (Number(settings.groups.general.refreshRate.value) / 1000)),
							'KB'
						)
					);
					netChart.data.datasets[1].data.push(
						convertBytes(
							Number(JSON.parse(d[2]).rx) *
								(1 / (Number(settings.groups.general.refreshRate.value) / 1000)),
							'KB'
						)
					);
					if (netChart.data.datasets[2].data.length < netChart.data.labels!.length) {
						netChart.data.datasets[2].data.push(
							Number(settings.groups.network.setUsageScale.value) * 0.8
						);
					}
					netChart.update();
				});
			}
		});

		timeoutId = setTimeout(() => {
			updateData();
		}, Number(settings.groups.general.refreshRate.value));
	};

	onMount(() => {
		fetch('/api/widget/resources', { method: 'GET' }).then(async (res) => {
			if (res.ok) {
				const data: string[] = await res.json();
				settings.groups.network.networkInterface.options = data.map((networkIf) => {
					return { value: networkIf, label: networkIf };
				});
				settings.groups.network.networkInterface.value = data[0];
			}
		});

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

		memChart = new Chart(memCanvas?.getContext('2d')!, {
			data: {
				labels: [],
				datasets: [
					{ label: 'Total', data: [] },
					{ label: 'Memory', data: [] },
					{ label: 'Swap', data: [] }
				]
			},
			type: 'line',
			options: {
				scales: {
					y: {
						beginAtZero: true,
						title: {
							display: true,
							text: 'Memory Usage'
						}
					}
				},
				animation: false
			}
		});

		netChart = new Chart(netCanvas?.getContext('2d')!, {
			data: {
				labels: [],
				datasets: [
					{ label: 'Upload', data: [] },
					{ label: 'Download', data: [] },
					{
						label: '80% marker',
						data: [],
						pointStyle: false
					}
				]
			},
			type: 'line',
			options: {
				scales: {
					y: {
						beginAtZero: true,
						max: Number(settings.groups.network.setUsageScale.value),
						title: {
							display: true,
							text: 'Network Usage (KB/s)'
						}
					}
				},
				animation: false
			}
		});

		netChart.data.datasets[2].data = [
			Number(settings.groups.network.setUsageScale.value) * 0.8,
			Number(settings.groups.network.setUsageScale.value) * 0.8
		];

		// start intervals
		updateData();

		return () => {
			cpuChart.destroy();
			memChart.destroy();
			netChart.destroy();
			clearTimeout(timeoutId);
		};
	});
</script>

<div class="flex h-full w-full flex-wrap">
	<div class="">
		<canvas bind:this={cpuCanvas}></canvas>
	</div>
	<div class="">
		<canvas bind:this={memCanvas}></canvas>
	</div>
	<div class="">
		<canvas bind:this={netCanvas}></canvas>
	</div>
</div>
