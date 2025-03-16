<script lang="ts">
	import { onMount } from 'svelte';

	let { media }: { media: MediaStream } = $props();
	let video: HTMLVideoElement | null = null;

	$effect(() => {
		if (video) {
			video.srcObject = media;
		}
	});
</script>

<!-- svelte-ignore a11y_media_has_caption -->
<video bind:this={video} playsinline autoplay muted></video>
