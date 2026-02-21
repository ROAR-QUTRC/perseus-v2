<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Topic monitor';
	export const description = 'Monitor the content and vitals of ROS topics';
	export const group = 'ROS';
	export const isRosDependent = true;

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			general: {
				maxErrorPercent: {
					type: 'number',
					value: '10',
					description:
						'This is used to calculate the colours used in the table. The for larger numbers values with greater error will be closer to green.'
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
				refreshTopics: {
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
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import { ScrollArea } from '$lib/components/ui/scroll-area/index';
	import * as ROSLIB from 'roslib';
	import { onMount, untrack } from 'svelte';
	import _ from 'lodash';
	import JsonTree from '$lib/components/jsonTree.svelte';
	import Button from '$lib/components/ui/button/button.svelte';


	// Widget logic goes here
	let monitors = $state<
		{
			topic: string;
			topicFound: boolean;
			listener: ROSLIB.Topic<object> | null;
			lastData: object;
			targetFrequency: number;
			currentFrequency: number;
			lastMessage: number;
			deadTime: number | null;
		}[]
	>([]);

	$effect(() => {
		if (!getRosConnection()) {
			untrack(() => {
				settings.groups.newMonitor.topic.options = [];
				monitors.forEach((monitor) => {
					monitor.listener?.unsubscribe();
				});
				monitors = [];
			});
		} else {
			untrack(() => {
				innit();
			});
		}
	});

	const innit = () => {
		const ros = getRosConnection()
		if (!ros) return;
		
		settings.groups.newMonitor.topic.value = '';

		ros.getTopics(
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
				if (monitors.find((monitor) => monitor.topic === topic)) continue; // skip is monitor is added
				addMonitor(topic, Number(frequency), true);
			}
		}
	};

	const onMessage = (message: object, topic: string) => {
		const index = monitors.findIndex((monitor) => monitor.topic === topic);
		if (index === -1) {
			console.error('How are we subscribed to a topic with no monitor?');
			return;
		}

		monitors[index].lastData = message

		// update frequency
		monitors[index].currentFrequency = 1000 / (Date.now() - monitors[index].lastMessage);
		monitors[index].lastMessage = Date.now();
	};

	const registerSettings = (topic: string, listener: ROSLIB.Topic<object> | null, frequency: number, topicFound: boolean) => {
		// add monitor to state
		monitors.push({
			topic: topic,
			topicFound: topicFound,
			listener: listener,
			lastData: {},
			targetFrequency: frequency,
			currentFrequency: 0,
			deadTime: null,
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
			},
			deleteMonitor: {
				type: 'button',
				action: () => {
					// unsubscribe from topic
					const index = monitors.findIndex((monitor) => monitor.topic === topic);
					if (index === -1) {
						console.error('How are we subscribed to a topic with no monitor?');
						return 'Monitor not found';
					}
					monitors[index].listener?.unsubscribe();
					monitors.splice(index, 1);
					// remove settings node
					delete settings.groups[topic];
					// update readonly config
					const config = settings.groups.newMonitor.config.value!.split(',');
					const topicIndex = config.findIndex((config) => config.split('@')[0] === topic);
					if (topicIndex === -1) return 'Monitor not found in config';
					config.splice(topicIndex, 1);
					settings.groups.newMonitor.config.value = config.join(',');

					return `${topic} monitor deleted`;
				}
			}
		};
	}

	const addMonitor = (topic: string | undefined, frequency: number, loadedFromConfig: boolean, shouldRegisterSettings: boolean = true) => {
		const ros = getRosConnection();
		if (!ros) return 'ROS not connected';

		if (topic === '' || topic === undefined) return 'No topic selected';

		if (shouldRegisterSettings)
			// prevent duplicate monitors
			for (const monitor of monitors) {
				if (monitor.topic === topic) return 'Monitor already exists';
			}

		let listener: ROSLIB.Topic<object> | null = null;

		if (shouldRegisterSettings) {
			const index = monitors.findIndex((monitor) => monitor.topic === topic);
			if (index !== -1) {
				monitors[index].listener?.removeAllListeners();
				monitors[index].listener?.unsubscribe();
			}
		}

		// Find the topic type
		ros.getTopicType(
			topic,
			(topicType) => {
				if (topicType) {
					// subscribe to ros topic
					listener = new ROSLIB.Topic<object>({
						ros: ros,
						name: topic,
						messageType: topicType
					});

					listener.removeAllListeners();
					listener.unsubscribe();
					listener.subscribe((message) => onMessage(message, topic));
				}

				// if this is a new monitor, add it to the config
				if (!loadedFromConfig) 
					settings.groups.newMonitor.config.value += topic + '@' + frequency + ',';

				if (shouldRegisterSettings)
					registerSettings(topic, listener, frequency, topicType !== '');

				if (topicType !== '') {
					// if we're loading from config, we need to register the settings after we've found the topic type
					const index = monitors.findIndex((monitor) => monitor.topic === topic);
					if (index === -1) {
						console.error('How are we subscribed to a topic with no monitor?');
						return 'Monitor not found';
					}
					monitors[index].topicFound = true;
					monitors[index].listener = listener!;
				}
			},
			(error) => {
				console.error('Error getting topic type', error);
			}
		);

		return 'Monitor created';
	};

	const getColor = (currentFrequency: number, targetFrequency: number) => {
		const error = currentFrequency - targetFrequency;
		if (error >= 0) return 'green' // on or above target frequency
		const maxError = Number(settings.groups.general.maxErrorPercent.value);
		if (Math.abs(error / targetFrequency) * 100 > maxError) return 'red'
		else return 'green'
	}

	let interval: NodeJS.Timeout | null = null;
	const MESSAGE_TIMEOUT_MS = 1000;

	onMount(() => {
		// add action for buttons
		settings.groups.newMonitor.createMonitor.action = (): string => {
			return addMonitor(settings.groups.newMonitor.topic.value, 10, false);
		};

		settings.groups.newMonitor.refreshTopics.action = (): string => {
			innit();
			return 'Topics refreshed';
		};

		// Find dead monitors
		interval = setInterval(() => {
			const now = Date.now();
			for (const monitor of monitors) {
				if (now - monitor.lastMessage > MESSAGE_TIMEOUT_MS) {
					monitor.currentFrequency = 0;
					monitor.deadTime = now - monitor.lastMessage;
				}
				else {
					monitor.deadTime = null;
				}
			}
		}, 100)

		return () => {
			// Cleanup
			settings.groups.newMonitor.topic.options = [];

			if (interval)
				clearInterval(interval);

			for (const monitor of monitors) {
				monitor.listener?.unsubscribe();
			}
		};
	});
</script>

<ScrollArea class="h-full -m-2" orientation="vertical">
	<div class="relative h-full w-full">
		<table class="w-full table-auto">
			<thead>
				<tr class="border-b">
					<th class="p-2 min-w-[200px]">Topic</th>
					<th class="border-l p-2">Last Data</th>
				</tr>
			</thead>
			<tbody>
				{#each monitors as monitor}
					<tr class="border border-l-0 border-x-0">
						<td class="p-2 flex flex-col">
							<p class="w-full text-center mb-2"><b>{monitor.topic}</b></p>
						
							<p
								class="rounded-[10px] text-border py-1 px-4 transition-colors text-center w-fit mx-auto"
								style:background-color={getColor(monitor.currentFrequency, monitor.targetFrequency)}
							>
								{monitor.currentFrequency.toFixed(1)}Hz
							</p>
							<p class="opacity-50 text-center">
								Target: {monitor.targetFrequency}Hz (&PlusMinus;{((Number(settings.groups.general.maxErrorPercent.value) / 100 * monitor.targetFrequency) as number).toFixed(2)}Hz)
							</p>
							{#if !monitor.topicFound}
								<p class="text-red-500 text-center">Topic not found</p>
							{:else if monitor.deadTime !== null}
								<p class="text-red-500 text-center">No message received for {(monitor.deadTime / 1000).toFixed(2)}s</p>
							{/if}
						</td>
						<td class="max-h-[200px] w-full overflow-hidden border-l p-2">
							{#if monitor.topicFound}
								<JsonTree data={monitor.lastData}/>
							{:else}
								<Button class="ml-[50%] -translate-x-[50%]" onclick={() => addMonitor(monitor.topic, monitor.targetFrequency, true, false)}>Reconnect</Button>
							{/if}
						</td>
					</tr>
				{/each}
			</tbody>
		</table>
		{#if monitors.length === 0}
			<p class="w-full border border-t-0 p-2 text-center">Add a monitor in the widget settings.</p>
		{/if}
	</div>
</ScrollArea>