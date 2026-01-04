import { io, Socket } from 'socket.io-client';
import config from './config.json';
import fs, { watch } from 'node:fs';
import { networkInterfaces } from 'os';
import { ChildProcessWithoutNullStreams, spawn } from 'node:child_process';

type videoTransformType =
	| 'none'
	| 'clockwise'
	| 'counterclockwise'
	| 'rotate-180'
	| 'horizontal-flip'
	| 'vertical-flip'
	| 'upper-left-diagonal'
	| 'upper-right-diagonal'
	| 'automatic';

interface CameraEventType {
	type: 'camera';
	action:
		| 'group-description'
		| 'kill'
		| 'request-groups'
		| 'request-stream'
		| 'group-terminated'
		| 'device-disconnect';
	data: {
		devices?: string[];
		resolution?: { width: number; height: number };
		transform?: videoTransformType;
		forceRestart?: boolean;
	};
}

const log = (message: string) => {
	console.log(`[Camera server] ${message}`);
};

log('Starting...');

// -------------------------------------
//        Web socket connection
// -------------------------------------

const socket: Socket = io(`http://${config.webserverIp}:${config.webserverPort}`, {
	reconnection: true,
	reconnectionDelay: 1000
});

socket.on('connect', () => {
	log('Connected to web server!');
});

let retryCount = 0;
socket.on('connect_error', (error) => {
	// retryCount++;
	// if (retryCount > 15) {
	// 	log(`Giving up. Please check the config.`);
	// 	process.exit(1);
	// }
	log(`Connection error, please check the config. Retrying...`);
});

// -------------------------------------
//           Get hardware info
// -------------------------------------

const formatDeviceName = (device: string): string =>
	// @ts-ignore
	device.replace('-video-index0', '').replace('usb-', '').replaceAll('_', ' ');

// Get the list of video devices
let videoDevices: string[] = fs
	.readdirSync('/dev/v4l/by-id')
	.filter((file) => file.endsWith('-index0')); // index 0 is the video and index 1 is the device metadata
videoDevices.forEach((device: string) => {
	log(`Found video device: ${formatDeviceName(device)}`);
});

// watch for changes in the video devices
let fsWatcher = watch('/dev/v4l/by-id', (eventType, filename) => {
	if (eventType === 'rename' && filename !== null && filename.endsWith('-index0')) {
		if (videoDevices.includes(filename)) {
			log(`Video device removed: ${formatDeviceName(filename)}`);
			videoDevices = videoDevices.filter((device) => device !== filename);

			// remove from gstInstances
			let index = gstInstances.findIndex((instance) => instance.device === filename);
			gstInstances[index].instance.kill();
			gstInstances.splice(index, 1);

			socket.send({
				type: 'camera',
				action: 'device-disconnect',
				data: { devices: [filename] }
			});
		} else {
			log(`Video device added: ${formatDeviceName(filename)}`);
			videoDevices.push(filename);

			initMessage.data.devices = videoDevices;
			socket.send(initMessage);
		}
	}
});

// -------------------------------------
//         Send initial message
// -------------------------------------

let initMessage: CameraEventType = {
	type: 'camera',
	action: 'group-description',
	data: { devices: videoDevices }
};
socket.send(initMessage);

// -------------------------------------
//        Handle socket events
// -------------------------------------

// Keep track of gstreamer instances since this info cannot be retrieved from the signalling server
let gstInstances: {
	device: string;
	resolution?: { width: number; height: number };
	transform?: videoTransformType;
	instance: ChildProcessWithoutNullStreams;
}[] = [];

const startStream = (
	device: string,
	resolution: { width: number; height: number },
	transform: videoTransformType
) => {
	let gstArgs = [
		'webrtcsink',
		'stun-server=NULL',
		'name=ws',
		`signaller::uri="ws://${config.webserverIp}:8443"`, // port is always default 8443
		`meta="meta,device=${device}"`
	];
	if (device === 'test') gstArgs.push('videotestsrc');
	else gstArgs.push('v4l2src', `device=/dev/v4l/by-id/${device}`);
	gstArgs.push(
		'!',
		`video/x-raw,width=${resolution.width},height=${resolution.height}`,
		'!',
		'videoconvert'
	);
	if (transform !== 'none') gstArgs.push('!', `videoflip`, `method=${transform}`);
	gstArgs.push('!', 'ws.');

	gstInstances.push({
		device: device,
		resolution: resolution,
		transform: transform,
		instance: spawn('gst-launch-1.0', gstArgs)
	});

	log('gst-launch1.0 ' + gstArgs.join(' '));

	gstInstances.at(-1)!.instance.stdout.pipe(process.stdout);
	gstInstances.at(-1)!.instance.stderr.pipe(process.stderr);
};

socket.on('camera-event', (event: CameraEventType) => {
	// Return if the target device is not owned by this server
	if (event.data && event.data.devices && !videoDevices.includes(event.data.devices[0])) return;
	switch (event.action) {
		case 'request-groups':
			initMessage.data.devices = videoDevices;
			socket.send(initMessage);
			break;
		case 'request-stream':
			// request stream
			log(
				`Requesting stream for device: ${formatDeviceName(event.data?.devices![0])} @ ${event.data?.resolution?.width}x${event.data?.resolution?.height} with transform: ${event.data?.transform}`
			);
			let index = gstInstances.findIndex(
				(instance) => instance.device === event.data?.devices?.[0]
			);

			if (event.data.forceRestart && index !== -1) {
				gstInstances[index].instance.kill();
				gstInstances.splice(index, 1);
			}

			let shouldStartStream = true;

			if (index !== -1) {
				if (!gstInstances[index]) {
					gstInstances.splice(index, 1);
				} else if (event.data.resolution && event.data.transform) {
					// If the stream is already running, check if we need to restart it
					if (
						event.data.resolution.width !== gstInstances[index].resolution?.width ||
						event.data.resolution.height !== gstInstances[index].resolution?.height ||
						event.data.transform !== gstInstances[index].transform
					) {
						// Restart the stream with new preferences
						gstInstances[index].instance.kill();
						gstInstances.splice(index, 1);
					} else {
						shouldStartStream = false;
					}
				} else {
					// device is already streaming and not preferences set
					break;
				}
			}

			if (shouldStartStream) {
				// Validate arguments
				if (!event.data.transform) event.data.transform = 'none';
				if (!event.data.resolution) event.data.resolution = { width: 320, height: 240 };
				startStream(event.data.devices![0], event.data.resolution, event.data.transform);
			}

			break;
		case 'group-description':
			// ignore self sent messages
			break;
		default:
			log(`Unknown action: ${event.action}`);
	}
});

// -------------------------------------
//    Gracefully shutdown the server
// -------------------------------------

process.on('SIGINT', () => {
	log('Shutting down...');

	initMessage.action = 'group-terminated';
	socket.send(initMessage);
	gstInstances.forEach((gstInstance) => {
		gstInstance.instance.kill();
	});
	fsWatcher.close();
	socket.close();
});
