// @ts-nocheck

import { spawn } from 'child_process';

let signallingServer = null;

export const cameraSocket = (socket, server) => {
	socket.on('message', (data) => {
		if (data.type === 'camera') {
			// start gstreamer signalling server on the first camera event
			if (signallingServer === null) {
				signallingServer = spawn('gst-webrtc-signalling-server');
			}
			server.emit('camera-event', data);
		}
	});
};
