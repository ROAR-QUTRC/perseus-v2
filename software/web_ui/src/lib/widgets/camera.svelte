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
      var blob = new Blob([ab], {type: imageFormat});
      return blob;
    }

	let canvas = $state<HTMLCanvasElement>();
    onMount(() => {
		const ctx = canvas!.getContext('2d');
        cameraTopic.subscribe((message: any) => {
			
			if (canvas !== undefined && ctx !== null) {
				createImageBitmap(dataURItoBlob("image/jpeg", message.data)).then((data) => {
					canvas!.width = data.width;
					canvas!.height = data.height;
					ctx.drawImage(data, 0, 0);
					data.close();
				ctx.beginPath();
				ctx.fillStyle = 'red';
				ctx.rect(20, 20, 20, 20);
				ctx.rect(60, 20, 20, 20);
				ctx.rect(20, 60, 10, 10);
				ctx.rect(70, 60, 10, 10);
				ctx.rect(30, 70, 40, 10);
				ctx.fill();
				});
			}

        });

        return () => {
            cameraTopic.unsubscribe();
        };
    });
</script>

<canvas bind:this={canvas}></canvas>