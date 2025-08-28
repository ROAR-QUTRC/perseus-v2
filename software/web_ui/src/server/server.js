import express from "express";
import { createServer } from "http";
import { Server } from "socket.io";

import { handler } from "../../build/handler.js";
import { resourceMonitor } from "./scripts/resourceMonitorSocket.js";
import { cameraSocket } from "./scripts/camerasSocket.js";
import { hostname } from "os";

const port = 3000;
const app = express();
const server = createServer(app);

const io = new Server(server);

let clientCount = 0;

io.on("connection", (socket) => {
  clientCount++;
  console.log(`Client connected. ${clientCount} clients connected`);

  // functions that act as websocket end points go here

  socket.on("disconnect", () => {
    clientCount--;
    console.log(`Client disconnected. ${clientCount} clients connected`);
  });

  cameraSocket(socket, io);
});

resourceMonitor(io);

app.use(handler);

server.listen(port, () => {
  console.log(
    "Beep boop! Server is running on http://" + hostname() + ".local:" + port,
  );
});
