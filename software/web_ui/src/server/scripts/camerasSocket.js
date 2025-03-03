// @ts-nocheck

export const cameraSocket = (socket, server) => {
  socket.on("message", (data) => {
    if (data.type === "camera") {
      server.emit("camera-event", data);
    }
  });
};
