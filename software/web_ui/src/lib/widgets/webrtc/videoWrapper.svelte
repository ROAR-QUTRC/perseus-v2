<script lang="ts">
	import Button from '$lib/components/ui/button/button.svelte';
	import { faEllipsis } from '@fortawesome/free-solid-svg-icons';
	import Fa from 'svelte-fa';
	import { peerConnections } from './signalHandler.svelte';
	import type { ConfigType, videoTransformType } from '../rtcVideo.svelte';
	import * as Select from '$lib/components/ui/select/index';

	let {
		device,
		config,
		onVideoClose,
		onVideoRestart,
		onVideoSettingsChange
	}: {
		device: string;
		config: ConfigType;
		onVideoClose: (device: string) => void;
		onVideoRestart: (device: string) => void;
		onVideoSettingsChange: (device: string, config: ConfigType) => void;
	} = $props();
	let video = $state<HTMLVideoElement | null>(null);

	let isSettingsOpen = $state<boolean>(false);

	$effect(() => {
		if (video) {
			video.srcObject = peerConnections[device].track;
		}
	});

	const openSettings = () => {
		isSettingsOpen = !isSettingsOpen;
	};

	let resolution = $state<string>(
		`${config.resolution.width}x${config.resolution.height}` || '320x240'
	);

	let transform = $state<string>(config.transform || 'none');

	const onValueChange = () => {
		isSettingsOpen = false;

		const [width, height] = resolution.split('x').map(Number);
		const newConfig: ConfigType = {
			name: config.name,
			resolution: { width, height },
			transform: transform as videoTransformType
		};
		onVideoSettingsChange(device, newConfig);
	};

	const closeButtonHandler = () => {
		onVideoClose(device);
		isSettingsOpen = false;
	};
</script>

<div class="relative grid h-full place-content-center">
	<!-- svelte-ignore a11y_media_has_caption -->
	{#if peerConnections[device].track}
		<video bind:this={video} playsinline autoplay muted></video>
	{:else if !peerConnections[device].online}
		<p class="m-auto w-[60%] text-center">
			Waiting for {peerConnections[device].name} to come online...
		</p>
	{:else}
		<p class="text-center">Connecting to {peerConnections[device].name}...</p>
	{/if}
	{#if isSettingsOpen}
		<div
			class="bg-card absolute left-0 top-0 flex h-full w-full flex-col items-center justify-center bg-opacity-80"
		>
			<Select.Root type="single" bind:value={resolution} {onValueChange}>
				<Select.Trigger class="mb-2 w-fit"><p class="pr-2">{resolution}</p></Select.Trigger>
				<Select.Content>
					<Select.Item value="320x240">320x240</Select.Item>
					<Select.Item value="640x480">640x480</Select.Item>
					<Select.Item value="1280x720">1280x720</Select.Item>
					<Select.Item value="1920x1080">1920x1080</Select.Item>
				</Select.Content>
			</Select.Root>
			<Select.Root type="single" bind:value={transform} {onValueChange}>
				<Select.Trigger class="mb-2 w-fit"><p class="pr-2">{transform}</p></Select.Trigger>
				<Select.Content>
					<Select.Item value="none">none</Select.Item>
					<Select.Item value="clockwise">clockwise</Select.Item>
					<Select.Item value="counterclockwise">counterclockwise</Select.Item>
					<Select.Item value="rotate-180">rotate-180</Select.Item>
					<Select.Item value="horizontal-flip">horizontal-flip</Select.Item>
					<Select.Item value="vertical-flip">vertical-flip</Select.Item>
					<Select.Item value="upper-left-diagonal">upper-left-diagonal</Select.Item>
					<Select.Item value="upper-right-diagonal">upper-right-diagonal</Select.Item>
					<Select.Item value="automatic">automatic</Select.Item>
				</Select.Content>
			</Select.Root>
			<div>
				<Button size="sm" variant="outline" class="mr-2" onclick={() => onVideoRestart(device)}
					>Restart</Button
				>
				<Button size="sm" onclick={closeButtonHandler}>Close</Button>
			</div>
		</div>
	{/if}
	<p class="bg-card absolute left-1 top-1 rounded-[4px] bg-opacity-60 px-2 py-1">
		{peerConnections[device].name}
	</p>
	<button
		onclick={openSettings}
		class="bg-card absolute right-1 top-1 h-[32px] rounded-[4px] bg-opacity-60"
	>
		<Fa icon={faEllipsis} class="cursor-pointer px-2 text-[20px]" />
	</button>
</div>
