export const connectToSignallingServer = (ip: string): WebSocket => {
	const ws = new WebSocket(`ws://${ip}:8443`);

	// auto reconnect on error
	ws.onerror = (event: any) => {
		setTimeout(() => {
			connectToSignallingServer(ip);
		}, 200);
	};

	ws.onmessage = (event: any) => {
		const data = JSON.parse(event.data);
		console.log(data);
	};

	return ws;
};

const send = (ws: WebSocket, data: any) => {
	if (!ws.OPEN) return;

	ws.send(JSON.stringify(data));
};
