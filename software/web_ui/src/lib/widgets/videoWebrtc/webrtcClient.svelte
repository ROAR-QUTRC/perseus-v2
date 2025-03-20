<script lang="ts">
	let {
		ip,
		groupName,
		cameras,
		cameraNames
	}: { ip: string; groupName: string; cameras: string[]; cameraNames: Record<string, string> } =
		$props();

	import { onMount } from 'svelte';
	import VideoWrapper from './videoWrapper.svelte';

	let ws: WebSocket | null = null;
	let peerConnections = $state<
		Record<
			string,
			{
				sessionId: string;
				name: string;
				connection: RTCPeerConnection | null;
				track: MediaStream | null;
			}
		>
	>({});
	let peerId = $state<string | null>(null);

	const wsSend = (data: any) => {
		if (!ws) return;
		// console.log('Sending:', data);
		ws.send(JSON.stringify(data));
	};

	const connectToSignallingServer = () => {
		ws = new WebSocket(`ws://${ip}:${8443}`);
		ws.onerror = (event) => {
			// if there is a websocket error just try again
			setTimeout(() => connectToSignallingServer(), 200);
			// console.log('[WS Error] -', event);
		};
		ws.onmessage = (event) => {
			const data = JSON.parse(event.data);
			// console.log(data);
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
						if (cameras.includes(data.meta.name)) {
							peerConnections[data.peerId] = {
								sessionId: '',
								name: data.meta.name,
								connection: null,
								track: null
							};

							wsSend({
								type: 'startSession',
								peerId: data.peerId
							});
						}
					}
					break;
				case 'list':
					if (data.producers.length > 0) {
						console.log('Producers:', data.producers);
						console.log('Cameras:', cameras);
						data.producers.forEach((producer: any) => {
							if (cameras.includes(producer.meta.name)) {
								// only that have not been added already
								if (!peerConnections[producer.id])
									peerConnections[producer.id] = {
										sessionId: '',
										name: producer.meta.name,
										connection: null,
										track: null
									};
							}
						});
						Object.keys(peerConnections).forEach((connection) => {
							if (peerConnections[connection].connection === null) {
								wsSend({
									type: 'startSession',
									peerId: connection
								});
							}
						});
						console.log('Remote ID:', data.producers[0].meta);
					}
					break;
				case 'sessionStarted':
					// When we are told that a new session is started create a new peer connection and set event handlers
					peerConnections[data.peerId] = {
						sessionId: data.sessionId,
						name: peerConnections[data.peerId]?.name || 'Unknown',
						connection: new RTCPeerConnection(),
						track: null
					};

					peerConnections[data.peerId].connection!.onicecandidate = (event) => {
						// console.log('onicecandidate', event);
						if (event.candidate && data.sessionId !== null) {
							wsSend({
								type: 'peer',
								sessionId: data.sessionId,
								ice: event.candidate.toJSON()
							});
						}
					};

					peerConnections[data.peerId].connection!.ontrack = (event) => {
						// console.log('ontrack', event);
						peerConnections[data.peerId].track = new MediaStream([event.track]);
					};
					break;
				case 'peer':
					// Peer messages are for negotiating the connection

					// get the producer ID
					const remoteId: string | undefined = Object.keys(peerConnections).find(
						(key) => peerConnections[key].sessionId === data.sessionId
					);

					if (!remoteId) {
						console.error('No remote ID found for session ID:', data.sessionId);
						return;
					}

					if (peerConnections[remoteId].connection === null) {
						console.error('No connection found for remote ID:', remoteId);
						return;
					}

					// acknowledge sdp and ice negotiation
					if (data.sdp) {
						peerConnections[remoteId].connection
							.setRemoteDescription(data.sdp)
							.then(() => {
								return peerConnections[remoteId].connection!.createAnswer();
							})
							.then((answer) => {
								return peerConnections[remoteId].connection!.setLocalDescription(answer);
							})
							.then(() => {
								wsSend({
									type: 'peer',
									sessionId: data.sessionId,
									sdp: peerConnections[remoteId].connection!.localDescription!.toJSON()
								});
							});
					} else if (data.ice) {
						peerConnections[remoteId].connection.addIceCandidate(new RTCIceCandidate(data.ice));
					} else {
						console.log('Unknown peer message:', data);
					}
					break;

				case 'endSession':
					// remove stream when session ends
					const sessionToEnd: string | undefined = Object.keys(peerConnections).find(
						(key) => peerConnections[key].sessionId === data.sessionId
					);
					if (sessionToEnd) delete peerConnections[sessionToEnd!];
					break;

				default:
					console.log('unknown message', data);
			}
		};
	};

	// create or remove peerconnections when the cameras prop changes
	$effect(() => {
		const connections = Object.keys(peerConnections);
		for (const connection of connections) {
			if (!cameras.includes(peerConnections[connection].name)) {
				console.log('Removing:', peerConnections[connection]);
				peerConnections[connection].connection?.close();
				peerConnections[connection].track = null;
				delete peerConnections[connection];
			}
		}
	});

	onMount(() => {
		connectToSignallingServer();

		return () => {
			// if (peerConnection) peerConnection.close();
			// close peer connections
			Object.keys(peerConnections).forEach((key) => {
				peerConnections[key].connection?.close();
			});
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
</script>

<!-- FOR DEBUGGING -->
<!-- <p>{ip}:{port}</p>
<p>Client ID: {peerId}</p> -->
<strong class="ml-2">{groupName}</strong>

<div class="m-1 mb-3 flex w-fit flex-row flex-wrap overflow-hidden rounded-[4px]">
	{#each Object.keys(peerConnections) as peer, i}
		{#if peerConnections[peer].track}
			<VideoWrapper
				media={peerConnections[peer].track}
				name={cameraNames[peerConnections[peer].name]}
			/>
		{/if}
	{:else}
		<p class="ml-2">Waiting for streams...</p>
	{/each}
</div>
