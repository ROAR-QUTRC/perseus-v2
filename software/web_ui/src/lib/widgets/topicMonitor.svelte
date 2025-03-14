<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Topic monitor';
	export const description = 'Monitor the content and vitals of ROS topics';
	export const group = 'ROS';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			general: {
				maxFrequencyError: {
					type: 'number',
					value: '10',
					description:
						'This is used to caluculate the colours used in the table. The for larger numbers values with greater error will be closer to green.'
				}
			},
			newMonitor: {
				topic: {
					type: 'select',
					options: []
				},
				createMonitor: {
					type: 'button'
				},
				config: {
					type: 'text',
					value: '',
					disabled: true
				}
			}
		}
	});
</script>

<script lang="ts">
	// TODO: make this use isConnected instead of ros.value === null
	import { isConnected, ros } from '$lib/scripts/ros.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import ROSLIB from 'roslib';
	import { onMount, untrack } from 'svelte';

	// Widget logic goes here
	let monitors = $state<
		{
			topic: string;
			listener: ROSLIB.Topic;
			lastData: string;
			targetFrequency: number;
			currentFrequency: number;
			lastMessage: number;
		}[]
	>([]);

	let rosConnected = $state<boolean>(false);
	$effect(() => {
		if (!isConnected()) {
			untrack(() => {
				rosConnected = false;
				settings.groups.newMonitor.topic.options = [];
				monitors.forEach((monitor) => {
					monitor.listener.unsubscribe();
				});
				monitors = [];
			});
		} else {
			rosConnected = true;
			innit();
		}
	});

	const innit = () => {
		if (ros.value === null) return;

		settings.groups.newMonitor.topic.value = '';

		ros.value.getTopics(
			(topics) => {
				const topicOptions = topics.topics.map((topic) => {
					return {
						value: topic,
						label: topic
					};
				});
				settings.groups.newMonitor.topic.options = topicOptions;
			},
			(error) => {
				console.error('Error getting topics', error);
			}
		);

		// load monitors from config
		let config: string | string[] | undefined = settings.groups.newMonitor.config.value;
		if (config !== '' && config !== undefined) {
			config = config.split(',');
			for (const monitor of config) {
				const [topic, frequency] = monitor.split('@');
				addMonitor(topic, Number(frequency), true);
			}
		}
	};

	const onMessage = (message: ROSLIB.Message, topic: string) => {
		const index = monitors.findIndex((monitor) => monitor.topic === topic);
		if (index === -1) {
			console.error('How are we subscribed to a topic with no monitor?');
			return;
		}
		monitors[index].lastData = JSON.stringify(message);

		// update frequency
		monitors[index].currentFrequency = 1000 / (Date.now() - monitors[index].lastMessage);
		monitors[index].lastMessage = Date.now();
	};

	const addMonitor = (topic: string | undefined, frequency: number, loadedFromConfig: boolean) => {
		if (ros.value === null) return 'ROS not connected';

		if (topic === '' || topic === undefined) return 'No topic selected';

		// prevent duplicate monitors
		for (const monitor of monitors) {
			if (monitor.topic === topic) return 'Monitor already exists';
		}

		// subscribe to ros topic
		const listener = new ROSLIB.Topic({
			ros: ros.value,
			name: topic,
			messageType: 'std_msgs/String'
		});

		listener.subscribe((message) => onMessage(message, topic));

		// if this is a new monitor, add it to the config
		if (!loadedFromConfig) settings.groups.newMonitor.config.value += topic + '@' + frequency + ',';

		// add monitor to state
		monitors.push({
			topic: topic,
			listener: listener,
			lastData: '',
			targetFrequency: frequency,
			currentFrequency: 0,
			lastMessage: Date.now()
		});

		// create settings node for monitor
		settings.groups[topic] = {
			expectedFrequency: {
				type: 'number',
				value: String(frequency)
			},
			updateMonitor: {
				type: 'button',
				action: () => {
					const frequency = settings.groups[topic].expectedFrequency.value;
					if (frequency === '' || frequency === undefined) return 'Invalid frequency';
					const index = monitors.findIndex((monitor) => monitor.topic === topic);
					if (index === -1) {
						console.error('How are we subscribed to a topic with no monitor?');
						return 'Monitor not found';
					}
					monitors[index].targetFrequency = Number(frequency);

					// update readonly config
					const config = settings.groups.newMonitor.config.value!.split(',');
					const topicIndex = config.findIndex((config) => config.split('@')[0] === topic);
					if (topicIndex === -1) return 'Monitor not found in config';
					config[topicIndex] = topic + '@' + frequency;
					settings.groups.newMonitor.config.value = config.join(',');

					return 'Monitor updated';
				}
			}
		};

		return 'Monitor created';
	};
	function getColor(value: number) {
		//value from 0 to 1
		value = Math.abs(value);
		if (value > 1) value = 1;
		if (value < 0) value = 0;
		var hue = ((1 - value) * 120).toString(10);
		return ['hsl(', hue, ',100%,50%)'].join('');
	}
	function getColorValue(value: number) {
		//value from 0 to 1
		value = Math.abs(value);
		if (value > 1) value = 1;
		if (value < 0) value = 0;
		var hue = ((1 - value) * 120).toString(10);
		return value;
	}

	onMount(() => {
		innit();

		// add action for button
		settings.groups.newMonitor.createMonitor.action = (): string => {
			return addMonitor(settings.groups.newMonitor.topic.value, 10, false);
		};

		return () => {
			// Cleanup
			settings.groups.newMonitor.topic.options = [];

			for (const monitor of monitors) {
				monitor.listener.unsubscribe();
			}
		};
	});
</script>

<div class="relative h-full w-full">
	<div
		class="absolute left-0 top-0 flex h-full w-full items-center justify-center bg-card"
		class:hidden={rosConnected}
	>
		<div class="absolute left-[50%] top-[50%] w-[80%] -translate-x-[50%] -translate-y-[50%]">
			<p class="text-center text-2xl">No ROS Connection found.</p>
			<p class="text-center">Make sure the rosbridge is running and the client is connected.</p>
		</div>
	</div>
	<table class="w-full table-auto">
		<thead>
			<tr class="border">
				<th class="border p-2">Topic</th>
				<th class="border p-2">Last Data</th>
				<th class="min-w-[140px] border p-2">Frequency</th>
			</tr>
		</thead>
		<tbody>
			{#each monitors as monitor}
				<tr class="border">
					<td class="border p-2">{monitor.topic}</td>
					<td class="border p-2">{monitor.lastData}</td>
					<td class="min-w-[140px] border p-2 text-black"
						><p
							class="rounded-[10px] py-2 text-center"
							style:background-color={getColor(
								Math.abs(monitor.currentFrequency - monitor.targetFrequency) /
									Number(settings.groups.general.maxFrequencyError.value!)
							)}
						>
							{Math.round(monitor.currentFrequency * 100) / 100}Hz / {monitor.targetFrequency}Hz
						</p></td
					>
				</tr>
			{/each}
		</tbody>
	</table>
	{#if monitors.length === 0}
		<p class="w-full border border-t-0 p-2 text-center">Add a monitor in the widget settings.</p>
	{/if}
</div>
