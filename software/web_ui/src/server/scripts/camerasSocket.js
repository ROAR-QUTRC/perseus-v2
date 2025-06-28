// @ts-nocheck

import { spawn } from 'child_process';

let signallingServer = null;

export const cameraSocket = (socket, server) => {
	socket.on('message', (data) => {
		if (data.type === 'camera') {
			// start gstreamer signalling server on the first camera event
			if (signallingServer === null) {
				console.log('Starting gstreamer signalling server');
				signallingServer = spawn('gst-webrtc-signalling-server');
				signallingServer.stdout.pipe(process.stdout);
				signallingServer.stderr.pipe(process.stderr);
			}
			server.emit('camera-event', data);
		}
	});
};
