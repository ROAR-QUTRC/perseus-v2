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

		// fetch data
		fetch('/api/widget/resources', { method: 'GET' }).then((res) => {
			if (res.ok) {
				res.json().then((d) => {
					// CPU
					cpuChart.data.datasets[0].data.push(JSON.parse(d[0]).cpuUsage);
					cpuChart.data.labels?.push(new Date().toLocaleTimeString('en-GB'));
					cpuChart.update();

					// Memory
					memChart.options.scales!.y!.max = convertBytes(
						Number(JSON.parse(d[1]).total[0]),
						settings.groups.memory.sizeFormat.value!
					);
					memChart.data.labels?.push(new Date().toLocaleTimeString('en-GB'));

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
				});
			}
		});

		timeoutId = setTimeout(() => {
			updateData();
		}, Number(settings.groups.general.refreshRate.value));
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
				datasets: [{ label: 'Network usage', data: [] }]
			},
			type: 'line',
			options: {
				scales: {
					y: {
						beginAtZero: true,
						title: {
							display: true,
							text: 'Network Usage (KB/s)'
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
			memChart.destroy();
			netChart.destroy();
			clearTimeout(timeoutId);
		};
	});
</script>

<div class="flex">
	<canvas bind:this={cpuCanvas}></canvas>
	<canvas bind:this={memCanvas}></canvas>
	<canvas bind:this={netCanvas}></canvas>
</div>
