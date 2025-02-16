<script lang="ts">
	let {
		ip,
		port,
		groupName,
		cameras
	}: { ip: string; port: number; groupName: string; cameras: any[] } = $props();

	import { onMount } from 'svelte';
	import VideoWrapper from './videoWrapper.svelte';

	let ws: WebSocket | null = null;
	let peerConnection: RTCPeerConnection | null = new RTCPeerConnection();

	let peerId = $state<string | null>(null);
	let remoteId = $state<string | null>(null);
	let callSessionId: string | null = null;

	let tracks = $state<MediaStream[]>([]);

	const wsSend = (data: any) => {
		if (!ws) return;
		ws.send(JSON.stringify(data));
	};

	peerConnection.onicecandidate = (event) => {
		// console.log('onicecandidate', event);
		if (event.candidate && callSessionId !== null) {
			wsSend({
				type: 'peer',
				sessionId: callSessionId,
				ice: event.candidate.toJSON()
			});
		}
	};

	const logger = (event: any) => {
		console.log(event.type, '->', event);
	};

	// peerConnection.onconnectionstatechange = logger
	// peerConnection.onicecandidateerror = logger;

	peerConnection.ontrack = (event) => {
		// console.log('ontrack', event);
		tracks.push(new MediaStream([event.track]));
	};

	const connectToSignallingServer = () => {
		ws = new WebSocket(`ws://${ip}:${port}`);
		ws.onerror = (event) => {
			// if there is a websocket error just try again
			setTimeout(() => connectToSignallingServer(), 200);
			// console.log('[WS Error] -', event);
		};
		ws.onmessage = (event) => {
			// console.log(event);
			const data = JSON.parse(event.data);
			switch (data.type) {
				case 'welcome':
					peerId = data.peerId;
					break;
				case 'peerStatusChanged':
					if (data.roles.includes('listener')) {
						wsSend({
							type: 'list'
						});
					}
					if (data.roles.includes('producer')) {
						remoteId = data.peerId;
					}
					break;
				case 'list':
					if (data.producers.length > 0) {
						remoteId = data.producers[0].id;
					}
					break;
				case 'sessionStarted':
					callSessionId = data.sessionId;
					break;
				case 'peer':
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
	};

	onMount(() => {
		connectToSignallingServer();

		return () => {
			if (peerConnection) peerConnection.close();
			if (ws) ws.close();
		};
	});

	$effect(() => {
		if (peerId) {
			wsSend({
				type: 'setPeerStatus',
				roles: ['listener'],
				meta: {
					name: 'gst-stream'
				}
			});
		}
	});

	$effect(() => {
		if (remoteId) {
			wsSend({
				type: 'startSession',
				peerId: remoteId
			});
		}
	});
</script>

<!-- FOR DEBUGGING -->
<!-- <p>{ip}:{port}</p>
<p>Client ID: {peerId}</p>
<p>Remote ID: {remoteId}</p> -->
<strong class="ml-2">{groupName}</strong>

<div class="m-1 mb-3 flex w-fit flex-row flex-wrap overflow-hidden rounded-[4px]">
	{#each tracks as track, i}
		<div class="relative">
			<VideoWrapper media={track} />
			<p class="absolute bottom-1 left-1 rounded-[4px] bg-card bg-opacity-60 px-2 py-1">
				{cameras[i].name}
			</p>
		</div>
	{/each}
</div>
