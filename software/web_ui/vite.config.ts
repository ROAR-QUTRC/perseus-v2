import { sveltekit } from '@sveltejs/kit/vite';
import { defineConfig, type ViteDevServer } from 'vite';

import { Server } from 'socket.io';
import { resourceMonitor } from './src/server/scripts/resourceMonitorSocket';

let clientCount: number = 0;

export const webSocketServer = {
	name: 'webSocketServer',
	configureServer(server: ViteDevServer) {
		if (!server.httpServer) return;

		const io = new Server(server.httpServer);

		io.on('connection', (socket) => {
			clientCount++;
			console.log(`Client connected. ${clientCount} clients connected`);

			// functions that act as websocket end points go here

			socket.on('disconnect', () => {
				clientCount--;
				console.log(`Client disconnected. ${clientCount} clients connected`);
			});
		});

		resourceMonitor(io);
	}
};

export default defineConfig({
	plugins: [sveltekit(), webSocketServer]
});
