import { sveltekit } from "@sveltejs/kit/vite";
import { defineConfig, type ViteDevServer } from "vite";

import { Server } from "socket.io";
import { resourceMonitor } from "./src/server/scripts/resourceMonitorSocket";

import dgram from "node:dgram";
// @ts-ignore
import parseRtpPacket from "simple-rtp-parser";

let clientCount: number = 0;

function dataURItoBlob(imageFormat: string, data: any) {
  console.log("Byte string: ", data);
  console.log("Byte string length: ", data.length);

  // write the bytes of the string to an ArrayBuffer
  var ab = new ArrayBuffer(data.length);
  console.log("ArrayBuffer 1: ", ab);
  var ia = new Uint8Array(ab);

  console.log("ArrayBuffer 2: ", ab);

  // write the ArrayBuffer to a blob, and you're done
  var blob = new Blob([ab], { type: imageFormat });
  console.log("Blob: ", blob);
  return blob;
}

export const webSocketServer = {
  name: "webSocketServer",
  configureServer(server: ViteDevServer) {
    if (!server.httpServer) return;

    const io = new Server(server.httpServer);

    io.on("connection", (socket) => {
      clientCount++;
      console.log(`Client connected. ${clientCount} clients connected`);

      // functions that act as websocket end points go here

      socket.on("disconnect", () => {
        clientCount--;
        console.log(`Client disconnected. ${clientCount} clients connected`);
      });
    });

    resourceMonitor(io);

    // UDP video streams
    const udpServer = dgram.createSocket("udp4");

    udpServer.on("error", (err) => {
      console.log(`server error:\n${err.stack}`);
    });

    udpServer.on("message", (msg, rinfo) => {
      // console.log(`server got: ${msg} from`, rinfo);
      // command: gst-launch-1.0 v4l2src device=/dev/video0 ! image/jpeg, width=320, height=240, framerate=30/1 ! udpsink host=0.0.0.0 port=8123
      // size: 22984
      // console.log(`server got: ${msg.toString('base64')} bytes from`, rinfo, '\n');

      const packet = parseRtpPacket(msg);
      console.log(
        (packet.payload as Buffer<ArrayBufferLike>).toString("base64"),
        rinfo,
        "\n",
      );

      io.emit(`video`, { raw: msg.toString("base64"), info: rinfo });
    });

    udpServer.on("listening", () => {
      const address = udpServer.address();
      console.log(`server listening ${address.address}:${address.port}`);
    });

    udpServer.bind(8123);
  },
};

export default defineConfig({
  plugins: [sveltekit(), webSocketServer],
});
