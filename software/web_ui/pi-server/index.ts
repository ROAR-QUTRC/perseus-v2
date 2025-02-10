import { io, Socket } from 'socket.io-client';
import config from './config.json';
import { spawn } from 'child_process';

const socket: Socket = io(`http://${config.webServer.ip}:${config.webServer.port}`, {
	reconnection: true,
	reconnectionDelay: 1000
});

let gstreamerInstance = spawn('gst-launch-1.0', [
	'webrtcsink',
	'run-signalling-server=true',
	'stun-server=NULL',
	'name=ws',
	'videtestsrc',
	'!',
	'ws.'
]);

gstreamerInstance.stdout.on('data', (data) => {
	console.log(`stdout: ${data}`);
});

socket.on('connect', () => {
	console.log('Connected!');
});

socket.on('connect_error', (error) => {
	console.log('Connection error, please check the config. Retrying...');
});

console.log('Starting camera client...');
