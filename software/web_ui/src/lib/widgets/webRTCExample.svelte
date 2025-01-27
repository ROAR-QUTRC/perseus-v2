<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'WebRTC example';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	/*
	gstreamer webrtcsink example:

	run these commands in separate terminals:
	~$ 											gst-launch-1.0 webrtcsink name=ws meta="meta,name=gst-stream" libcamerasrc ! ws.
	~/gst-plugins-rs/net/webrtc/signalling $    WEBRTCSINK_SIGNALLING_SERVER_LOG=debug cargo run --bin gst-webrtc-signalling-server -- --port 8443
	~/gst-plugins-rs/net/webrtc/gstwebrtc-api $ npm start
	*/

	import { onMount } from 'svelte';
</script>

<!-- <img src={image} alt="img" /> -->

<!-- <canvas bind:this={canvas}></canvas> -->

<!-- svelte-ignore a11y_missing_attribute -->
<iframe src="http://10.1.1.74:8888/mystream" scrolling="no" class="h-[100%] w-[100%]"></iframe>
