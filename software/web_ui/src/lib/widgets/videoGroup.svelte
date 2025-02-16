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

	let devices = $state<{ ip: string; port: number; groupName: string; cameras: object[] }[]>([]);

	// $inspect(devices);

	socket.on('camera-event', (event) => {
		console.log(event);
		switch (event.action) {
			case 'init':
				// TODO: check if device already has ip
				if (devices.find((device) => device.ip === event.data.ip) === undefined)
					devices.push({
						ip: event.data.ip,
						port: 8443,
						groupName: event.data.groupName,
						cameras: event.data.cameras
					});
				break;
			case 'get-streams':
				break;
			case 'kill':
				const index = devices.findIndex((device) => device.ip === event.data.ip);
				console.log(index);
				devices.splice(index, 1);
				console.log('Stream HAS ENDED!!!');
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
	<WebrtcClient
		ip={device.ip}
		port={device.port}
		groupName={device.groupName}
		cameras={device.cameras}
	/>
{:else}
	<p>
		No active streams. Check that peripheral servers are running and that you are on the correct
		network.
	</p>
{/each}
