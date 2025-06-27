<script lang="ts">
	let { media, name }: { media: MediaStream; name: string } = $props();
	let video: HTMLVideoElement | null = null;

	$effect(() => {
		if (video) {
			video.srcObject = media;
		}
	});
</script>

<div class="relative h-fit">
	<!-- svelte-ignore a11y_media_has_caption -->
	<video bind:this={video} playsinline autoplay muted></video>
	<p class="absolute bottom-1 left-1 rounded-[4px] bg-card bg-opacity-60 px-2 py-1">{name}</p>
</div>
