import { io, Socket } from "socket.io-client";
import config from "./config.json";
import { spawn } from "child_process";
import { networkInterfaces } from "os";
import fs from "node:fs";

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
    | "stream"
    | "request-groups"
    | "request-stream"
    | "producer-ready"
    | "group-terminated";
  data: {
    ip?: string;
    groupName?: string;
    // cameraName?: string;
    resolution?: { width: number; height: number };
    transform?: videoTransformType;
    device?: string;
    cameras?: string[];
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

// get the ip address of the server
if (networkIf) ip = networkIf[0].address;

// get a list of cameras
let cameras: string[] = fs
  .readdirSync("/dev/v4l/by-id")
  .filter((file) => file.endsWith("-index0")); // index 0 is the video and index 1 is the device metadata
cameras.forEach((camera: string) => {
  console.log(
    `[Camera server] Found camera: ${camera.replace("-video-index0", "").replace("usb-", "").replaceAll("_", " ")}`,
  );
});

let initMessage: CameraEventType = {
  type: "camera",
  action: "group-description",
  data: { groupName: config.groupName, ip: ip, cameras: cameras },
};

if (ip) {
  console.log(`[Camera server] IP address: ${ip}`);
  socket.send(initMessage);
} else {
  console.error("[Camera server] Could not get IP address");
}

// start the signalling server
let signallingServer = spawn("gst-webrtc-signalling-server");
signallingServer.stdout.pipe(process.stdout);

let gstInstances: {
  device: string;
  resolution: { width: number; height: number };
  transform: videoTransformType;
  instance: any;
}[] = [];

// webrtcsink stun-server=NULL name=ws meta="meta,name=usb-Quanta_HP_Wide_Vision_HD_Camera_01.00.00-video-index0" v4l2src device=/dev/v4l/by-id/usb-Quanta_HP_Wide_Vision_HD_Camera_01.00.00-video-index0 ! video/x-raw, width=320, height=240 ! videoconvert ! videoflip method=clockwise ! ws.
const startStream = (
  device: string,
  resolution: { height: number; width: number },
  transform: videoTransformType,
) => {
  // start the camera stream
  let gstArgs = ["webrtcsink", "stun-server=NULL", "name=ws"];
  gstArgs.push(`meta="meta,name=${device}"`); // assign a name to the stream
  if (device === "test") gstArgs.push("videotestsrc");
  else gstArgs.push("v4l2src", `device=/dev/v4l/by-id/${device}`);
  gstArgs.push(
    `!`,
    `video/x-raw,`,
    `width=${resolution.width},`,
    `height=${resolution.height}`,
    `!`,
    `videoconvert`,
  );

  if (transform) gstArgs.push(`!`, `videoflip`, `method=${transform}`);

  gstArgs.push(`!`, "ws.");

  gstInstances.push({
    device: device,
    resolution: resolution,
    transform: transform,
    instance: spawn("gst-launch-1.0", gstArgs),
  });

  // console.log('[Camera server] Instance command:' + gstArgs.join(' '));

  gstInstances[gstInstances.length - 1].instance.stdout.pipe(process.stdout);
  gstInstances[gstInstances.length - 1].instance.stderr.pipe(process.stderr);
};

// listen for camera events
socket.on("camera-event", (event: CameraEventType) => {
  if (event.data && event.data.groupName !== config.groupName) return;
  switch (event.action) {
    case "request-groups":
      socket.send(initMessage);
      break;
    case "request-stream":
      console.log("[Camera server] request-stream ", event.data);
      // check if the camera is already streaming
      let index = gstInstances.findIndex(
        (instance) => instance.device === event.data.device,
      );

      if (index !== -1) {
        // if the stream is currently running
        if (event.data.resolution) {
          if (
            (gstInstances[index].resolution.width !==
              event.data.resolution.width &&
              gstInstances[index].resolution.height !==
                event.data.resolution.height) ||
            gstInstances[index].transform !== event.data.transform
          ) {
            // if the resolution is different restart the stream
            console.log(
              `[Camera server] Changing resolution of ${event.data.device} from ${gstInstances[index].resolution.width}x${gstInstances[index].resolution.height} to ${event.data.resolution.width}x${event.data.resolution.height} with transform: ${event.data.transform}`,
            );
            gstInstances[index].instance.kill();
            gstInstances.splice(index, 1);
            startStream(
              event.data.device!,
              event.data.resolution,
              event.data.transform ? event.data.transform : "none",
            );
            break;
          }
        } else {
          console.log(
            `[Camera server] ${event.data.device} is already streaming`,
          );
          break;
        }
      } else {
        // start the stream
        console.log(`[Camera server] Starting stream for ${event.data.device}`);
        startStream(
          event.data.device!,
          event.data.resolution
            ? event.data.resolution
            : { width: 320, height: 240 },
          event.data.transform ? event.data.transform : "none",
        );
      }
      break;
    case "kill":
      let instanceIndex = gstInstances.findIndex(
        (instance) => instance.device === event.data.device,
      );
      if (instanceIndex !== -1) {
        console.log(`[Camera server] Killing ${event.data.device}`);
        gstInstances[instanceIndex].instance.kill();
        gstInstances.splice(instanceIndex, 1);
      }
      break;
    case "producer-ready":
    case "group-description":
      break;

    default:
      console.log("[Camera server] Received unhandled event: ", event);
  }
});
console.log("[Camera server] Started");

process.on("SIGINT", () => {
  // tell clients to kill this group
  initMessage.action = "group-terminated";
  socket.send(initMessage);
  signallingServer.kill();
  gstInstances.forEach((instance) => {
    instance.instance.kill();
  });
  socket.close();
  console.log("[Camera server] Exiting...");
  process.exit(1);
});
