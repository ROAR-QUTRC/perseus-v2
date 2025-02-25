<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';
	import { onMount } from 'svelte';

	export const name = 'Teleop';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	// import { ros } from '$lib/scripts/ros.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	// import ROSLIB from 'roslib';

	// Widget logic goes here

	const gamePadConnected = (event: any) => {
		console.log(event);
	};

	onMount(() => {
		console.log(navigator.getGamepads());
	});
</script>

<svelte:window ongamepadconnected={gamePadConnected} />

<p>New component</p>
