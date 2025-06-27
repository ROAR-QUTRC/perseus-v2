<script lang="ts">
	import { faEllipsis } from '@fortawesome/free-solid-svg-icons';
	import Fa from 'svelte-fa';

	let { media, name }: { media: MediaStream; name: string } = $props();
	let video = $state<HTMLVideoElement | null>(null);

	let isSettingsOpen = $state<boolean>(false);

	$effect(() => {
		if (video) {
			video.srcObject = media;
		}
	});

	const openSettings = () => {
		isSettingsOpen = !isSettingsOpen;
	};
</script>

<div class="relative h-fit">
	{#if !isSettingsOpen}
		<!-- svelte-ignore a11y_media_has_caption -->
		<video bind:this={video} playsinline autoplay muted></video>
	{:else}
		<div>Settings hehe</div>
	{/if}
	<p class="bg-card absolute left-1 top-1 rounded-[4px] bg-opacity-60 px-2 py-1">{name}</p>
	<button
		onclick={openSettings}
		class="bg-card absolute right-1 top-1 h-[32px] rounded-[4px] bg-opacity-60"
	>
		<Fa icon={faEllipsis} class="cursor-pointer px-2 text-[20px]" />
	</button>
</div>
