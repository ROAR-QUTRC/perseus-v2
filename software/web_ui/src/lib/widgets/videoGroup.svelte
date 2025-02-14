<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Video canvas - WebRTC';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	import WebrtcClient from '$lib/widgets/videoWebrtc/webrtcClient.svelte';
	import { io, type Socket } from 'socket.io-client';
	import { onMount } from 'svelte';

	let socket: Socket = io();

	let devices = $state<{ ip: string; port: number; cameras: object[] }[]>([]);

	socket.on('camera-event', (event) => {
		switch (event.action) {
			case 'init':
				// TODO: check if device already has ip
				devices.push({ ip: event.data.location.ip, port: 8443, cameras: event.data.cameras });
				break;
			case 'get-streams':
				break;
			default:
				console.log(event);
				break;
		}
	});

	onMount(() => {
		socket.send({ type: 'camera', action: 'get-streams' });
	});
</script>

{#each devices as device}
	<WebrtcClient ip={device.ip} port={device.port} cameras={device.cameras} />
{/each}
