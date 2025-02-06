<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Video - WebRTC';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			registerCamera: {
				name: {
					type: 'text',
					description: 'Enter the name of the stream',
					value: 'Camera 1'
				},
				host: {
					type: 'text',
					description: 'Select the ip of the host that the stream is hosted on',
					options: [],
					value: '10.1.1.133'
				},
				port: {
					type: 'number',
					description: 'Enter the port of the host that the stream is hosted on',
					value: '8443'
				},
				StartSession: {
					type: 'button',
					description: 'Start a new session',
					action: () => {
						if (Object.keys(settings.groups).includes(settings.groups.registerCamera.name.value!)) {
							return 'Session already exists';
						}

						settings.groups.registerCamera.cameraConfig.value += `${settings.groups.registerCamera.name.value}@${
							settings.groups.registerCamera.host.value
						}:${settings.groups.registerCamera.port.value},`;

						return 'Created session';
					}
				},
				debugMode: {
					type: 'switch',
					description: 'Enable debug mode',
					value: 'false'
				},
				cameraConfig: {
					// cameraname@ip:port,cameraName@ip:port
					type: 'readonly',
					description: 'Read only, for internal use',
					value: ''
				}
			}
		}
	});
</script>

<script lang="ts">
	import { untrack, onMount } from 'svelte';
	import { type WebRtcSessionType } from '$lib/widgets/videoWebrtc/webrtc';
	import VideoWrapper from '$lib/widgets/videoWebrtc/videoWrapper.svelte';
	import panzoom from 'panzoom';

	$effect(() => {
		let config: string | string[] | undefined = settings.groups.registerCamera.cameraConfig.value;
		// only run when there is at least one camera
		if (config && config.length > 0) {
			config = config.split(',').filter((c) => c.length > 0);

			// untrack prevents infinite loops due to reassignment of reactive variables
			untrack(() => {
				// ensure there is a setting group for each camera
				const groups = Object.keys(settings.groups).filter((group) => group !== 'registerCamera');

				// Close old connections
				groups.forEach((group) => {
					if (
						(config as string[]).filter((camera) => {
							return camera.startsWith(group);
						}).length === 0
					)
						delete settings.groups[group];
				});

				connections = connections.filter((conn) => {
					return (config as string[]).includes(`${conn.name}@${conn.ip}:${conn.port}`);
				});

				// ensure all active connections are open
				(config as string[]).forEach((camera) => {
					const [cameraName, address] = camera.split('@');

					// Create new groups for each camera
					if (!groups.includes(cameraName) || !settings.groups[cameraName].closeSession.action) {
						settings.groups[cameraName] = {
							closeSession: {
								type: 'button',
								description: 'Close the session',
								action: () => {
									settings.groups.registerCamera.cameraConfig.value =
										settings.groups.registerCamera.cameraConfig.value!.replace(camera + ',', '');
									return 'Session closed';
								}
							}
						};
					}

					const [ip, port] = address.split(':');
					ensureConnection(cameraName, ip, port);
				});
			});
		}
	});

	let connections = $state<WebRtcSessionType[]>([]);

	const ensureConnection = (name: string, ip: string, port: string) => {
		let i = connections.findIndex((conn) => conn.name === name);

		// create new connection and save the index
		if (i === -1) {
			i =
				connections.push({
					name,
					ip,
					port,
					tracks: []
				}) - 1;
		}

		// check connection
		if (!connections[i].signallingSocket) {
			connections[i].signallingSocket = new WebSocket(`ws://${ip}:${port}`);
			connections[i].signallingSocket!.onmessage = (message: any) => {
				onSignal(connections[i], message);
			};
			console.log('Signalling socket created');
		}

		if (!connections[i].peerConnection) {
			connections[i].peerConnection = new RTCPeerConnection();
			connections[i].peerConnection!.onicecandidate = (event) => {
				onIceCandidate(connections[i], event);
			};
			connections[i].peerConnection!.ontrack = (event) => {
				onTrack(connections[i], event);
			};
			console.log('Peer connection created');
		}

		console.log('Ensuring connection', name, ip, port);
	};

	const onSignal = (session: WebRtcSessionType, message: any) => {
		const data = JSON.parse(message.data);
		// this.debug('recieved:', data);
		switch (data.type) {
			case 'welcome':
				session.clientId = data.peerId;
				// console.log(session.name, `- client id: ${session.clientId}`);
				// register as a listener
				session.signallingSocket!.send(
					JSON.stringify({
						type: 'setPeerStatus',
						roles: ['listener'],
						meta: { name: session.name + Date.now().toString() }
					})
				);
				break;
			case 'peerStatusChanged':
				if (data.roles.includes('listener')) {
					// console.log(session.name, '- new listener:', data.peerId);
					session.signallingSocket!.send(
						JSON.stringify({
							type: 'list'
						})
					);
				}
				if (data.roles.includes('producer')) {
					// console.log(session.name, '- new producer:', data.peerId);
					session.producerId = data.peerId;
					if (session.producerId) initCall(session);
				}
				break;
			case 'list':
				if (data.producers.length > 0) {
					session.producerId = data.producers[0].id;
					// console.log(session.name, '- Found produceers:', data.producers);
					if (session.producerId) initCall(session);
				}
			case 'sessionStarted':
				session.sessionId = data.sessionId;
				break;
			case 'peer':
				if (data.sdp) {
					session
						.peerConnection!.setRemoteDescription(data.sdp)
						.then(() => {
							return session.peerConnection!.createAnswer();
						})
						.then((answer) => {
							return session.peerConnection!.setLocalDescription(answer);
						})
						.then(() => {
							session.signallingSocket!.send(
								JSON.stringify({
									type: 'peer',
									sessionId: data.sessionId,
									sdp: session.peerConnection!.localDescription!.toJSON()
								})
							);
						});
				} else if (data.ice) {
					session.peerConnection!.addIceCandidate(new RTCIceCandidate(data.ice));
				} else {
					console.log(session.name, '- Unknown peer message:', data);
				}
				break;
			default:
				// console.log(session.name, '- recieved:', data);
				break;
		}
	};

	const initCall = (session: WebRtcSessionType) => {
		session.signallingSocket!.send(
			JSON.stringify({ type: 'startSession', peerId: session.producerId })
		);
	};

	const onIceCandidate = (session: WebRtcSessionType, event: any) => {
		// console.log('onicecandidate', event);
		if (event.candidate && session.sessionId !== null) {
			session.signallingSocket!.send(
				JSON.stringify({
					type: 'peer',
					sessionId: session.sessionId,
					ice: event.candidate.toJSON()
				})
			);
		}
	};

	const onTrack = (session: WebRtcSessionType, event: any) => {
		console.log(session.name, '- ontrack', event);
		session.tracks.push(new MediaStream([event.track]));
	};

	let instance;
	const initPanZoom = (node: HTMLElement) => {
		instance = panzoom(node, { maxZoom: 2, minZoom: 0.5 });
	};

	onMount(() => {
		return () => {
			connections.forEach((conn) => {
				conn.signallingSocket?.close();
				conn.peerConnection?.close();
			});
		};
	});
</script>

<div class="h-full w-full">
	<div class="flex flex-col bg-orange-400" use:initPanZoom>
		{#each connections as conn, i}
			<div class=" m-1 bg-slate-500">
				<p>Camera: {conn.name} ({i})</p>
				<p>Client id: {conn.clientId}</p>
				<p>Producer id: {conn.producerId}</p>
				<p>Session id: {conn.sessionId}</p>
				<div class="flex flex-row">
					{#each conn.tracks as track}
						<VideoWrapper media={track} />
					{:else}
						<p>No tracks</p>
					{/each}
				</div>
			</div>
		{/each}
	</div>
</div>
