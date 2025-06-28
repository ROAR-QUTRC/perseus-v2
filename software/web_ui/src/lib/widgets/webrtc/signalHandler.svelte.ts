export let peerConnections = $state<
	Record<
		string,
		{
			sessionId: string;
			name: string;
			online: boolean;
			connection: RTCPeerConnection | null;
			track: MediaStream | null;
		}
	>
>({});
let peerId = $state<string | undefined>(undefined);
let producerDeviceMap = $state<Record<string, string>>({});

export let ws: WebSocket | null = null;

interface SignalMessageType {
	type:
		| 'welcome'
		| 'setPeerStatus'
		| 'list'
		| 'peerStatusChanged'
		| 'startSession'
		| 'peer'
		| 'sessionStarted'
		| 'endSession';
	peerId?: string;
	sessionId?: string;
	roles?: string[];
	meta?: {
		name?: string;
		device?: string;
	};
	producers?: Array<{ id: string; meta: { device: string } }>;
	ice?: any;
	sdp?: any;
}

export const connectToSignallingServer = (ip: string) => {
	ws = new WebSocket(`ws://${ip}:${8443}`);
	ws.onopen = () => {
		console.log('[WS] Connected to signalling server');
	};
	ws.onerror = (event) => {
		// if there is a websocket error just try again
		setTimeout(() => connectToSignallingServer(ip), 200);
		console.log('[WS Error] -', event);
	};
	ws.onmessage = (event) => {
		const data: SignalMessageType = JSON.parse(event.data);
		switch (data.type) {
			case 'welcome':
				// Sent when the signalling server registers a new connection
				peerId = data.peerId;
				// Tell the signalling server that we are a listener
				if (peerId) {
					wsSend({
						type: 'setPeerStatus',
						roles: ['listener'],
						meta: {
							name: 'web-client'
						}
					} as SignalMessageType);
				}
				break;
			case 'peerStatusChanged':
				// Sent when a peer's status changes
				if (data.peerId === peerId && data.roles?.includes('listener')) {
					// If this device has been confirmed as a listener, request streams
					wsSend({ type: 'list' });
				} else if (data.roles?.includes('producer')) {
					// If a camera produces comes online after start up, request its stream
					console.log(Object.keys(peerConnections));
					if (Object.keys(peerConnections).includes(data.meta?.device!)) {
						peerConnections[data.meta?.device!].online = true;
						producerDeviceMap[data.peerId!] = data.meta?.device!;
						wsSend({
							type: 'startSession',
							peerId: data.peerId // camera peerId
						});
					}
				}
				break;
			case 'list':
				if (data.producers && data.producers.length > 0) {
					data.producers.forEach((producer) => {
						if (
							peerConnections[producer.meta.device] &&
							!peerConnections[producer.meta.device].online
						) {
							// If the producer is already in the list, just set it to online
							peerConnections[producer.meta.device].online = true;
							producerDeviceMap[producer.id] = producer.meta.device;
							wsSend({
								type: 'startSession',
								peerId: producer.id // camera peerId
							});
						}
					});
				}
				break;
			case 'sessionStarted':
				// When a session is started, we need to create a new RTCPeerConnection
				const targetDevice = producerDeviceMap[data.peerId!];

				peerConnections[targetDevice] = {
					sessionId: data.sessionId!,
					name: peerConnections[targetDevice].name,
					online: true,
					connection: new RTCPeerConnection(),
					track: null
				};

				peerConnections[targetDevice].connection!.onicecandidate = (event) => {
					if (event.candidate && data.sessionId !== null) {
						wsSend({
							type: 'peer',
							sessionId: data.sessionId,
							ice: event.candidate.toJSON()
						});
					}
				};

				peerConnections[targetDevice].connection!.ontrack = (event) => {
					peerConnections[targetDevice].track = new MediaStream([event.track]);
				};
				break;
			case 'peer':
				// Peer messages are for negotiating the connection

				// determine target device from the sessionId
				const peerDevice: string | undefined = Object.keys(peerConnections).find(
					(device) => peerConnections[device].sessionId === data.sessionId
				);

				if (!peerDevice) {
					console.warn('[WS] No peer device found for sessionId:', data.sessionId);
					return;
				}

				if (peerConnections[peerDevice].connection === null) {
					console.error('[WS] No connection found for peer device:', peerDevice);
					return;
				}

				// acknowledge sdp and ice negotiation
				if (data.sdp) {
					peerConnections[peerDevice].connection
						.setRemoteDescription(data.sdp)
						.then(() => {
							return peerConnections[peerDevice].connection!.createAnswer();
						})
						.then((answer) => {
							return peerConnections[peerDevice].connection!.setLocalDescription(answer);
						})
						.then(() => {
							wsSend({
								type: 'peer',
								sessionId: data.sessionId,
								sdp: peerConnections[peerDevice].connection!.localDescription!.toJSON()
							});
						});
				} else if (data.ice) {
					peerConnections[peerDevice].connection!.addIceCandidate(new RTCIceCandidate(data.ice));
				} else {
					console.error('[ws] Unknown peer message:', data);
				}
				break;
			case 'endSession':
				// Remove stream when session ends
				const deviceToRemove: string | undefined = Object.keys(peerConnections).find(
					(device) => peerConnections[device].sessionId === data.sessionId
				);
				if (deviceToRemove) {
					peerConnections[deviceToRemove].track = null;
					peerConnections[deviceToRemove].connection?.close();
					peerConnections[deviceToRemove].connection = null;
					peerConnections[deviceToRemove].sessionId = '';
					peerConnections[deviceToRemove].online = false;
				}
				break;
			default:
				console.log('[WS] Unknown message type:', data);
		}
	};
};

const wsSend = (data: SignalMessageType) => {
	if (!ws) return;
	// console.log('Sending:', data);
	ws.send(JSON.stringify(data));
};

export const getPeerId = () => {
	if (peerId) return peerId;
	return null;
};
