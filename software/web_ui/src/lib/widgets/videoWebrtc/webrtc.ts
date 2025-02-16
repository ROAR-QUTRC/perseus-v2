export interface WebRtcSessionType {
	name: string;
	ip: string;
	port: string;
	signallingSocket?: WebSocket;
	enableDebugging?: boolean;

	clientId?: string | null;
	producerId?: string | null;
	sessionId?: string | null;

	peerConnection?: RTCPeerConnection;
	tracks: MediaStream[];

	// onNewTrack: (
	// 	track: MediaStream,
	// 	sessionInfo: { clientId: string; producerId: string; sessionId: string }
	// ) => void;
}

export class WebRtcSession {
	name: string;
	ip: string;
	port: string;
	signallingSocket: WebSocket;
	enableDebugging: boolean;

	clientId: string | null = null;
	producerId: string | null = null;
	sessionId: string | null = null;

	peerConnection: RTCPeerConnection;

	onNewTrack:
		| ((
				track: MediaStream,
				sessionInfo: { clientId: string; producerId: string; sessionId: string }
		  ) => void)
		| null = null;

	constructor(name: string, ip: string, port: string, debug: boolean = false) {
		this.name = name;
		this.ip = ip;
		this.port = port;
		this.enableDebugging = debug;

		this.signallingSocket = new WebSocket(`ws://${ip}:${port}`);

		this.debug(`WebRtcSession: ${this.name} created`);

		this.signallingSocket.onmessage = this.onSignal;

		this.peerConnection = new RTCPeerConnection();
		this.peerConnection.onicecandidate = (event) => {
			this.debug('onicecandidate:', event);
			if (event.candidate && this.sessionId !== null) {
				this.sendSignal({
					type: 'peer',
					sessionId: this.sessionId,
					ice: event.candidate.toJSON()
				});
			}
		};

		this.peerConnection.ontrack = (event) => {
			this.debug('ontrack:', event);
			this.debug(this.peerConnection);
			if (this.onNewTrack)
				this.onNewTrack(new MediaStream([event.track]), {
					clientId: this.clientId!,
					producerId: this.producerId!,
					sessionId: this.sessionId!
				});
		};
	}

	private sendSignal = (message: any) => {
		if (this.signallingSocket.readyState === WebSocket.OPEN) {
			this.debug('sending:', message);
			this.signallingSocket.send(JSON.stringify(message));
		} else {
			this.debug(`SOCKET ERROR: socket not open`);
		}
	};

	private onSignal = (message: any) => {
		const data = JSON.parse(message.data);
		// this.debug('received:', data);
		switch (data.type) {
			case 'welcome':
				this.clientId = data.peerId;
				this.debug(`client id: ${this.clientId}`);
				// register as a listener
				this.sendSignal({ type: 'setPeerStatus', roles: ['listener'], meta: { name: this.name } });
				break;
			case 'peerStatusChanged':
				if (data.roles.includes('listener')) {
					this.debug('new listener:', data.peerId);
					this.sendSignal({
						type: 'list'
					});
				}
				if (data.roles.includes('producer')) {
					this.debug('new producer:', data.peerId);
					this.producerId = data.peerId;
					if (this.producerId) this.call();
				}
				break;
			case 'list':
				if (data.producers.length > 0) {
					this.producerId = data.producers[0].id;
					this.debug('Found produceers:', data.producers);
					if (this.producerId) this.call();
				}
			case 'sessionStarted':
				this.sessionId = data.sessionId;
				break;
			case 'peer':
				if (data.sdp) {
					this.peerConnection
						.setRemoteDescription(data.sdp)
						.then(() => {
							return this.peerConnection.createAnswer();
						})
						.then((answer) => {
							return this.peerConnection.setLocalDescription(answer);
						})
						.then(() => {
							this.sendSignal({
								type: 'peer',
								sessionId: data.sessionId,
								sdp: this.peerConnection.localDescription!.toJSON()
							});
						});
				} else if (data.ice) {
					this.peerConnection.addIceCandidate(new RTCIceCandidate(data.ice));
				} else {
					console.log('Unknown peer message:', data);
				}
				break;
			default:
				this.debug('received:', data);
				break;
		}
	};

	private call() {
		this.sendSignal({
			type: 'startSession',
			peerId: this.producerId
		});
	}

	private debug(...args: any[]) {
		if (this.enableDebugging) {
			for (let i = 0; i < args.length; i++) {
				console.log(`[WebRtc - ${this.name}]`, args[i]);
			}
		}
	}
}
