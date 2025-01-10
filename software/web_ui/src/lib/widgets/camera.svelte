<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Video stream';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	import { ros } from '$lib/scripts/ros.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import ROSLIB from 'roslib';

	let cameraTopic = new ROSLIB.Topic({
		ros: ros.value!,
		name: '/v4l2_camera/image_raw',
		messageType: 'sensor_msgs/msg/Image'
	});

	console.log(cameraTopic);

	cameraTopic.subscribe((message) => {
		cameraData = message;
		console.log(message);
	});

	let cameraData = $state()
</script>

<p>{cameraData}</p>
