<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'listener';
	// These properties are optional
	// export const description = 'Description of the widget goes here';
	export const group = 'ROS';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	import { ros } from '$lib/scripts/ros.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import ROSLIB from 'roslib';
	import { onMount } from 'svelte';

	// Widget logic goes here
	let data = $state('');
	let transmitData = $state('');

	const transmit = (e: Event) => {
		e.preventDefault();
		const publisher = new ROSLIB.Topic({
			ros: ros.value!,
			name: '/ros_to_can',
			messageType: 'std_msgs/String'
		});

		const message = new ROSLIB.Message({
			data: transmitData
		});

		publisher.publish(message);
		transmitData = '';
	};

	onMount(() => {
		const listener = new ROSLIB.Topic({
			ros: ros.value!,
			name: '/can_to_ros',
			messageType: 'std_msgs/String'
		});

		listener.subscribe((message: any) => {
			data = message.data;
			let parsed = '';
			try {
				parsed = JSON.parse(data);
				console.log(parsed);
			} catch (e) {}
		});
	});
</script>

<p>Data: {data}</p>

<form>
	<input type="text" bind:value={transmitData} />
	<button type="submit" onclick={transmit}>Send</button>
</form>
