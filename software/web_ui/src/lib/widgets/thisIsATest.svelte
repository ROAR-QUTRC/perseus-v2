<script lang="ts" module>
	import { ros } from '$lib/scripts/ros.svelte';
	// This is to expose the widget settings to the panel. Only put code here if you know why you are doing it!
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';
	import ROSLIB from 'roslib';

	export const name = 'New Widget';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			General: {
				SelectATopic: {
					type: 'select',
					description: 'Select a topic to view',
					options: []
				},
				RefreshTopicsList: {
					type: 'button',
					action: () => {
						if (ros.value?.isConnected) {
							ros.value.getTopics((topics) => {
								settings.groups.General.SelectATopic.options = topics.topics.map((topic) => {
									console.log(topic);
									return { value: topic, label: topic };
								});
							});
						}
						return 'success';
					}
				}
			}
		}
	});
</script>

<script lang="ts">
	let output = $state<string>('null');

	// $effect(() => {
	// 	if (ros.value?.isConnected) {
	// 		ros.value.getNodes((nodes) => {
	// 			nodes.forEach((node) => {
	// 				ros.value?.getNodeDetails(node, (details) => {
	// 					console.log(node, details);
	// 				});
	// 			});
	// 			// console.log(nodes);
	// 		});
	// 	}
	// });

	// let nodes = $derived.by<Array<string>>(() => {
	// 	if (ros.value?.isConnected) {
	// 		ros.value.getNodes((nodes) => {
	// 			return nodes;
	// 		});
	// 	}
	// 	return [];
	// });

	// $inspect(nodes);

	$effect(() => {
		if (ros.value?.isConnected) {
			ros.value?.getParams(
				(params) => {
					console.log('params', params);
				},
				(error) => {
					console.error(error);
				}
			);
		}
	});

	let listener = new ROSLIB.Topic({
		ros: ros.value!,
		name: '/turtle1/cmd_vel',
		messageType: 'geometry_msgs/msg/Twist'
	});
	// listener.subscribe((message) => {
	// 	console.log(message);
	// });

	const onkeydown = (e: KeyboardEvent) => {
		if (ros.value?.isConnected) {
			switch (e.key) {
				case 'w':
					listener.publish(new ROSLIB.Message({ linear: { x: 1 }, angular: { z: 0 } }));
					break;
				case 's':
					listener.publish(new ROSLIB.Message({ linear: { x: -1 }, angular: { z: 0 } }));
					break;
				case 'a':
					listener.publish(new ROSLIB.Message({ linear: { x: 0 }, angular: { z: 1 } }));
					break;
				case 'd':
					listener.publish(new ROSLIB.Message({ linear: { x: 0 }, angular: { z: -1 } }));
					break;
			}
		}
	};
</script>

<div>move with wasd</div>

<svelte:window {onkeydown} />
