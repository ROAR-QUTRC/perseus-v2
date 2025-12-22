<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Tutorial ROS Bridge Widget';
	// These properties are optional
	// export const description = 'Description of the widget goes here';
	export const group: WidgetGroupType = 'ROS';
	export const isRosDependent = true;

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte';
	import * as ROSLIB from 'roslib';
	import { onMount, untrack } from 'svelte';

	let message = $state<string>('');

	let topic = $state<ROSLIB.Topic | null>(null);

	$effect(() => {
		const ros = getRosConnection() as ROSLIB.Ros;
		untrack(() => {
			if (ros) {
				topic = new ROSLIB.Topic({
					ros: ros,
					name: '/topic',
					messageType: 'std_msgs/String'
				});

				topic.subscribe((topicMessage: any) => {
					message = topicMessage.data;
				});
			} else {
				if (topic) {
					topic.unsubscribe();
					topic = null;
				}
				message = '';
				console.error('We lost ros');
			}
		});
	});

	onMount(() => {
		return () => {
			if (topic) {
				topic.unsubscribe();
				topic = null;
			}
		};
	});
</script>

<p>Message: {message}</p>
