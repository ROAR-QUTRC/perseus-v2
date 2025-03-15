import { io, Socket } from "socket.io-client";
import config from "./config.json";
import { exec, spawn } from "child_process";
import { networkInterfaces } from "os";

interface CameraEventType {
  type: "camera";
  action:
    | "group-description"
    | "kill"
    | "request-groups"
    | "request-stream"
    | "producer-ready";
  data: {
    ip?: string;
    groupName?: string;
    cameraName?: string;
    resolution?: { width: number; height: number };
    device?: string;
  };
}

console.log("[Camera server] Starting...");
// connect to the web servers websocket
const socket: Socket = io(
  `http://${config.webServer.ip}:${config.webServer.port}`,
  {
    reconnection: true,
    reconnectionDelay: 1000,
  },
);

socket.on("connect", () => {
  console.log("[Camera server][Socket] Connected!");
});

socket.on("connect_error", (error) => {
  console.log(
    "[Camera server][Socket] Connection error, please check the config. Retrying...",
  );
});

// got this servers ip address
let net = networkInterfaces();
let networkIf = net[Object.keys(net)[1]];
let ip: string | undefined = undefined;

if (networkIf) ip = networkIf[0].address;

let initMessage: CameraEventType = {
  type: "camera",
  action: "group-description",
  data: { groupName: config.groupName, ip: ip },
};

if (ip) {
  console.log(`[Camera server] IP address: ${ip}`);
  socket.send(initMessage);
} else {
  console.error("[Camera server] Could not get IP address");
}

// start the signalling server
let signallingServer = spawn("gst-webrtc-signalling-server");
let gstInstances: {
  name: string;
  resolution: { width: number; height: number };
  instance: any;
}[] = [];

// listen for camera events
socket.on("camera-event", (event: CameraEventType) => {
  switch (event.action) {
    case "request-groups":
      socket.send(initMessage);
      break;
    case "request-stream":
      // check if the camera is already streaming
      let index = gstInstances.findIndex(
        (instance) => instance.name === event.data.cameraName,
      );
      if (index !== -1) {
        if (
          gstInstances[index].resolution.width ===
            event.data.resolution!.width &&
          gstInstances[index].resolution.height ===
            event.data.resolution!.height
        ) {
          break;
        } else {
          console.log(
            `[Camera server] Changing resolution of ${event.data.cameraName} from ${gstInstances[index].resolution.width}x${gstInstances[index].resolution.height} to ${event.data.resolution!.width}x${event.data.resolution!.height}`,
          );
          gstInstances[index].instance.kill();
          gstInstances.splice(index, 1);
        }
      }

      console.log(
        `[Camera server] Starting stream from ${event.data.device} at ${event.data.resolution!.width}x${event.data.resolution!.height}`,
      );

      // start the camera stream
      let gstArgs = ["webrtcsink", "stun-server=NULL", "name=ws"];
      gstArgs.push(
        `meta="meta,name=${event.data.cameraName!.split(" ").join("")}"`,
      ); // assign a name to the stream
      if (event.data.device === "test") gstArgs.push("videotestsrc");
      else gstArgs.push("v4l2src", `device=${event.data.device}`);
      gstArgs.push(
        `!`,
        `video/x-raw, width=${event.data.resolution!.width}, height=${event.data.resolution!.height}`,
        `!`,
        `videoconvert`,
        `!`,
        "ws.",
      );
      gstInstances.push({
        name: event.data.cameraName!,
        resolution: event.data.resolution!,
        instance: spawn("gst-launch-1.0", gstArgs),
      });

      // console.log('[Camera server] Instance command:' + gstArgs.join(' '));

      gstInstances[gstInstances.length - 1].instance.stdout.pipe(
        process.stdout,
      );
      gstInstances[gstInstances.length - 1].instance.stderr.pipe(
        process.stderr,
      );
      break;
    case "producer-ready":
    case "kill":
    case "group-description":
      break;

    default:
      console.log("[Camera server] Received unhandled event: ", event);
  }
});
console.log("[Camera server] Started");

// console.log('---- Starting signalling server ----');
// // let signallingServer = spawn('gst-webrtc-signalling-server');

// let gstArgs = ['webrtcsink', 'stun-server=NULL', 'name=ws'];

// console.log('---- Setting up cameras ----');
// config.cameras.forEach((camera) => {
// 	console.log(
// 		`[${camera.name}] - Adding stream from ${camera.device} at ${camera.minResolution.width}x${camera.minResolution.height}`
// 	);
// 	if (camera.device === 'libcamera') gstArgs.push('libcamerasrc');
// 	else if (camera.device === 'test') gstArgs.push('videotestsrc');
// 	else gstArgs.push('v4l2src', `device=${camera.device}`);
// 	gstArgs.push(
// 		`!`,
// 		`video/x-raw, width=${camera.minResolution.width}, height=${camera.minResolution.height}`,
// 		`!`,
// 		`videoconvert`,
// 		`!`
// 	);
// 	if (camera.device === 'libcamera') gstArgs.push('queue', '!');
// 	gstArgs.push('ws.');
// });

// socket.send({
// 	type: 'camera',
// 	action: 'init',
// 	data: {
// 		ip: ip,
// 		groupName: config.groupName,
// 		cameras: config.cameras
// 	}
// });

// console.log('---- Set up complete ----');

// // 'webrtcsink run-signalling-server=true stun-server=NULL name=ws v4l2src device=/dev/video0 ! video/x-raw, width=640, height=480 ! videoconvert ! ws. videotestsrc ! ws.'.split(' ')
// // let gstreamerInstance = spawn('gst-launch-1.0', gstArgs);

// // gstreamerInstance.stdout.pipe(process.stdout);
// // gstreamerInstance.stderr.pipe(process.stderr);

// socket.on('connect', () => {
// 	console.log('[Socket] - Connected!');
// });

// socket.on('connect_error', (error) => {
// 	console.log('[Socket] - Connection error, please check the config. Retrying...');
// });

// socket.on('camera-event', (event) => {
// 	switch (event.action) {
// 		case 'get-streams':
// 			socket.send({
// 				type: 'camera',
// 				action: 'init',
// 				data: {
// 					ip: ip,
// 					groupName: config.groupName,
// 					cameras: config.cameras
// 				}
// 			});
// 			break;
// 		case 'init': // ignore events sent by self
// 		case 'kill':
// 			break;
// 		default:
// 			console.log(event);
// 	}
// });

// console.log();

// // handle ctrl + c
// process.on('SIGINT', async (event) => {
// 	await new Promise(async (resolve) => {
// 		console.log('---- Releasing Cameras ----');
// 		// gstreamerInstance.kill();
// 		// signallingServer.kill();
// 		console.log('---- Closing websockets ----');
// 		socket.send({
// 			type: 'camera',
// 			action: 'kill',
// 			data: { ip: ip }
// 		});
// 		socket.close();
// 		console.log('---- Saying Goodbye ----');
// 		resolve(null);
// 	});
// 	console.log(event);
// });

process.on("SIGINT", () => {
  // tell clients to kill this group
  initMessage.action = "kill";
  socket.send(initMessage);
  signallingServer.kill();
  gstInstances.forEach((instance) => {
    instance.instance.kill();
  });
  socket.close();
  console.log("[Camera server] Exiting...");
  process.exit(1);
});
