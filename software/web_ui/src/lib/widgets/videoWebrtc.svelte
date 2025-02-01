<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'New Widget';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	import { onMount } from 'svelte';
	import Button from '$lib/components/ui/button/button.svelte';

	let ws: WebSocket | null = null;
	let peerConnection: RTCPeerConnection | null = new RTCPeerConnection({
		iceServers: [
			{
				urls: 'stun:stun.l.google.com:19302'
			}
		]
	});
	let peerId = $state<string | null>(null);
	let remoteId = $state<string | null>(null);
	let callSessionId: string | null = null;

	let remoteVideo = $state<null | HTMLVideoElement>(null);

	const wsSend = (data: any) => {
		if (!ws) return;
		console.log('Sending:', data);
		ws.send(JSON.stringify(data));
	};

	peerConnection.onicecandidate = (event) => {
		console.log('onicecandidate', event);
		if (event.candidate && callSessionId !== null) {
			wsSend({
				type: 'peer',
				sessionId: callSessionId,
				ice: event.candidate.toJSON()
			});
		}
	};

	peerConnection.ontrack = (event) => {
		console.log('ontrack', event);
		if (remoteVideo) {
			remoteVideo.srcObject = event.streams[0];
		}
	};

	onMount(() => {
		ws = new WebSocket('ws://10.1.1.74:8443');
		ws.onmessage = (event) => {
			// console.log(event);
			const data = JSON.parse(event.data);
			switch (data.type) {
				case 'welcome':
					peerId = data.peerId;
					break;
				case 'peerStatusChanged':
					console.log('Client status: ', data.roles);
					if (data.roles.includes('listener')) {
						wsSend({
							type: 'list'
						});
					}
					if (data.roles.includes('producer')) {
						console.log('New producer found:', data.peerId);
						remoteId = data.peerId;
					}
					break;
				case 'list':
					if (data.producers.length > 0) {
						console.log('Found producers:', data.producers);
						remoteId = data.producers[0].id;
					}
					break;
				case 'sessionStarted':
					console.log('Session started:', data);
					callSessionId = data.sessionId;
					break;
				case 'peer':
					console.log('Peer message:', data);
					if (data.sdp) {
						peerConnection
							.setRemoteDescription(data.sdp)
							.then(() => {
								return peerConnection.createAnswer();
							})
							.then((answer) => {
								return peerConnection.setLocalDescription(answer);
							})
							.then(() => {
								wsSend({
									type: 'peer',
									sessionId: data.sessionId,
									sdp: peerConnection.localDescription!.toJSON()
								});
							});
					} else if (data.ice) {
						peerConnection.addIceCandidate(new RTCIceCandidate(data.ice));
					} else {
						console.log('Unknown peer message:', data);
					}
					break;

				default:
					console.log('unknown message', data);
			}
		};

		return () => {
			if (ws) ws.close();
		};
	});

	const registerClient = () => {
		wsSend({
			type: 'setPeerStatus',
			roles: ['listener'],
			meta: {
				name: 'gst-stream'
			}
		});
	};

	const call = () => {
		wsSend({
			type: 'startSession',
			peerId: remoteId
		});
	};

	const test = () => {
		settings.groups = {
			...settings.groups,
			general: {
				testSetting: {
					type: 'number',
					description: 'Test setting',
					value: '42'
				}
			}
		};
	};
</script>

<p>Client ID: {peerId}</p>
<p>Remote ID: {remoteId}</p>

<Button onclick={registerClient}>Register</Button>
<Button onclick={call}>Call</Button>
<Button onclick={test}>Test</Button>

<!-- svelte-ignore a11y_media_has_caption -->
<video bind:this={remoteVideo} playsinline autoplay muted></video>
