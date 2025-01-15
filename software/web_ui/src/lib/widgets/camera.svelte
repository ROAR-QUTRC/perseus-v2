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
	import { onMount } from 'svelte';

	let cameraTopic = new ROSLIB.Topic({
		ros: ros.value!,
		name: '/image_raw/compressed',
		messageType: 'sensor_msgs/msg/CompressedImage'
	});

	onMount(() => {
		const canvas = document.getElementById('canvas') as HTMLCanvasElement;
		const ctx = canvas.getContext('2d');
		ctx?.clearRect(0, 0, canvas.width, canvas.height);

		cameraTopic.subscribe((message: any) => {
			let img = new Image();
			img.src = 'data:image/jpeg;base64,' + message.data;
			ctx?.drawImage(img, 0, 0);
			cameraTopic.unsubscribe();
		});

		return () => {
			cameraTopic.unsubscribe();
		};
	});

	let cameraData = $state();
</script>

<!-- <p class=" w-[50%] text-wrap">{cameraData}</p> -->
<!-- <img alt="" src="data:image/png;base64,{cameraData}" /> -->
<canvas id="canvas" width="640" height="480"></canvas>
