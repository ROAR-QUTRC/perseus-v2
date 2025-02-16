import { io, Socket } from 'socket.io-client';
import config from './config.json';
import { exec, spawn } from 'child_process';
import { group } from 'console';

const socket: Socket = io(`http://${config.webServer.ip}:${config.webServer.port}`, {
	reconnection: true,
	reconnectionDelay: 1000
});

let gstArgs = ['webrtcsink', 'run-signalling-server=true', 'stun-server=NULL', 'name=ws'];

console.log('---- Setting up cameras ----');
config.cameras.forEach((camera) => {
	console.log(
		`[${camera.name}] - Adding stream from ${camera.device} at ${camera.minResolution.width}x${camera.minResolution.height}`
	);
	if (camera.device === 'libcamera') gstArgs.push('libcamerasrc');
	else if (camera.device === 'test') gstArgs.push('videotestsrc');
	else gstArgs.push('v4l2src', `device=${camera.device}`);
	gstArgs.push(
		`!`,
		`video/x-raw, width=${camera.minResolution.width}, height=${camera.minResolution.height}`,
		`!`,
		`videoconvert`,
		`!`
	);
	if (camera.device === 'libcamera') gstArgs.push('queue', '!');
	gstArgs.push('ws.');
});
exec('hostname -I', (_, stdout, __) => {
	socket.send({
		type: 'camera',
		action: 'init',
		data: { ip: stdout.split(' ')[0], groupName: config.groupName, cameras: config.cameras }
	});
});
console.log('---- Set up complete ----');

// 'webrtcsink run-signalling-server=true stun-server=NULL name=ws v4l2src device=/dev/video0 ! video/x-raw, width=640, height=480 ! videoconvert ! ws. videotestsrc ! ws.'.split(' ')
let gstreamerInstance = spawn('gst-launch-1.0', gstArgs);

gstreamerInstance.stdout.pipe(process.stdout);
gstreamerInstance.stderr.pipe(process.stderr);

socket.on('connect', () => {
	console.log('[Socket] - Connected!');
});

socket.on('connect_error', (error) => {
	console.log('[Socket] - Connection error, please check the config. Retrying...');
});

socket.on('camera-event', (event) => {
	switch (event.action) {
		case 'get-streams':
			exec('hostname -I', (_, stdout, __) => {
				socket.send({
					type: 'camera',
					action: 'init',
					data: { ip: stdout.split(' ')[0], groupName: config.groupName, cameras: config.cameras }
				});
			});
			break;
		case 'init': // ignore events sent by self
		case 'kill':
			break;
		default:
			console.log(event);
	}
});

console.log();

// handle ctrl + c
process.on('SIGINT', async (event) => {
	await new Promise(async (resolve) => {
		console.log('---- Releasing Cameras ----');
		gstreamerInstance.kill();
		console.log('---- Closing websockets ----');
		await new Promise((resolve) =>
			exec('hostname -I', (_, stdout, __) => {
				socket.send({
					type: 'camera',
					action: 'kill',
					data: { ip: stdout.split(' ')[0] }
				});
				resolve(null);
			})
		);
		socket.close();
		console.log('---- Saying Goodbye ----');
		resolve(null);
	});
	console.log(event);
});
