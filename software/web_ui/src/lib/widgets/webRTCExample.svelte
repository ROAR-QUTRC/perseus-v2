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

	run these commands in seperate terminals:
	~$ 											gst-launch-1.0 webrtcsink name=ws meta="meta,name=gst-stream" libcamerasrc ! ws.
	~/gst-plugins-rs/net/webrtc/signalling $    WEBRTCSINK_SIGNALLING_SERVER_LOG=debug cargo run --bin gst-webrtc-signalling-server -- --port 8443
	~/gst-plugins-rs/net/webrtc/gstwebrtc-api $ npm start
	*/

	import { onMount } from 'svelte';
	import { io, Socket } from 'socket.io-client';

	function dataURItoBlob(imageFormat: string, dataURI: string) {
		// convert base64 to raw binary data held in a string
		// doesn't handle URLEncoded DataURIs - see SO answer #6850276 for code that does this
		var byteString = atob(dataURI);

		// write the bytes of the string to an ArrayBuffer
		var ab = new ArrayBuffer(byteString.length);
		var ia = new Uint8Array(ab);
		for (var i = 0; i < byteString.length; i++) {
			ia[i] = byteString.charCodeAt(i);
		}

		// write the ArrayBuffer to a blob, and you're done
		var blob = new Blob([ab], { type: imageFormat });
		return blob;
	}

	let canvas = $state<HTMLCanvasElement>();
	onMount(() => {
		const socket: Socket = io();
		const ctx = canvas!.getContext('2d');
		socket.on('video', async (message: { raw: any; info: any }) => {
			if (canvas !== undefined && ctx !== null) {
				createImageBitmap(dataURItoBlob('image/jpeg', message.raw)).then((data) => {
					canvas!.width = data.width;
					canvas!.height = data.height;
					ctx.drawImage(data, 0, 0);
					data.close();
					// ctx.beginPath();
					// ctx.fillStyle = 'red';
					// ctx.rect(20, 20, 20, 20);
					// ctx.rect(60, 20, 20, 20);
					// ctx.rect(20, 60, 10, 10);
					// ctx.rect(70, 60, 10, 10);
					// ctx.rect(30, 70, 40, 10);
					// ctx.fill();
				});
			}
		});

		return () => {};
	});
</script>

<!-- <img src={image} alt="img" /> -->

<canvas bind:this={canvas}></canvas>
