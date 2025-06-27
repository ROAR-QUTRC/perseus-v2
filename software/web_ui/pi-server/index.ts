import { io, Socket } from "socket.io-client";
import config from "./config.json";
import fs, { watch } from "node:fs";
import { networkInterfaces } from "os";
import { spawn } from "node:child_process";

type videoTransformType =
  | "none"
  | "clockwise"
  | "counterclockwise"
  | "rotate-180"
  | "horizontal-flip"
  | "vertical-flip"
  | "upper-left-diagonal"
  | "upper-right-diagonal"
  | "automatic";

interface CameraEventType {
  type: "camera";
  action:
    | "group-description"
    | "kill"
    | "request-groups"
    | "request-stream"
    | "group-terminated";
  data: {
    ip?: string;
    devices?: string[];
    properties?: {
      resolution?: { width: number; height: number };
      transform?: videoTransformType;
    };
  };
}

const log = (message: string) => {
  console.log(`[Camera server] ${message}`);
};

log("Starting...");

// -------------------------------------
//        Web socket connection
// -------------------------------------

const socket: Socket = io(
  `http://${config.webserverIp}:${config.webserverPort}`,
  {
    reconnection: true,
    reconnectionDelay: 1000,
  },
);

socket.on("connect", () => {
  log("Connected to web server!");
});

let retryCount = 0;
socket.on("connect_error", (error) => {
  retryCount++;
  if (retryCount > 15) {
    log(`Giving up. Please check the config.`);
    process.exit(1);
  }
  log(`Connection error, please check the config. Retrying...`);
});

// -------------------------------------
//           Get hardware info
// -------------------------------------

// Get this servers ip address
let net = networkInterfaces();
let networkIf = net[Object.keys(net)[1]];
let ip: string | undefined = networkIf ? networkIf[0].address : undefined;

const formatDeviceName = (device: string): string =>
  // @ts-ignore
  device.replace("-video-index0", "").replace("usb-", "").replaceAll("_", " ");

// Get the list of video devices
let videoDevices: string[] = fs
  .readdirSync("/dev/v4l/by-id")
  .filter((file) => file.endsWith("-index0")); // index 0 is the video and index 1 is the device metadata
videoDevices.forEach((device: string) => {
  log(`Found video device: ${formatDeviceName(device)}`);
});

// watch for changes in the video devices
watch("/dev/v4l/by-id", (eventType, filename) => {
  if (
    eventType === "rename" &&
    filename !== null &&
    filename.endsWith("-index0")
  ) {
    if (videoDevices.includes(filename)) {
      log(`Video device removed: ${formatDeviceName(filename)}`);
      videoDevices = videoDevices.filter((device) => device !== filename);
    } else {
      log(`Video device added: ${formatDeviceName(filename)}`);
      videoDevices.push(filename);
    }
    // TODO - send an event to the web server with the updated list of devices
  }
});

// -------------------------------------
//         Send initial message
// -------------------------------------

let initMessage: CameraEventType = {
  type: "camera",
  action: "group-description",
  data: { ip: ip, devices: videoDevices },
};

if (ip) {
  log(`IP address: ${ip}`);
  socket.send(initMessage);
} else {
  log(`Could not get IP address`);
  process.exit(1);
}

// -------------------------------------
//       Start signalling server
// -------------------------------------

// const signallingServer = spawn('gst-webrtc-signalling-server');
// signallingServer.stdout.pipe(process.stdout);

// signallingServer.stderr.on('data', (data) => {
// 	log(` [ERROR] Signalling server: ${data}`);
// });

// -------------------------------------
//        Handle socket events
// -------------------------------------

// Keep track of gstreamer instances since this info cannot be retrieved from the signalling server
let gstInstances: {
  device: string;
  resolution?: { width: number; height: number };
  transform?: videoTransformType;
  instance: any;
}[] = [];

socket.on("camera-event", (event: CameraEventType) => {
  // Return if the target device is not owned by this server
  if (event.data.devices && !videoDevices.includes(event.data.devices[0]))
    return;
  switch (event.action) {
    case "group-description":
      // ignore self sent messages
      break;
    default:
      log(`Unknown action: ${event.action}`);
  }
});

// -------------------------------------
//    Gracefully shutdown the server
// -------------------------------------

process.on("SIGINT", () => {});
