import ROSLIB from 'roslib';

export let ros = $state<{ value: ROSLIB.Ros | null }>({
	value: null
});

let connected = $state<boolean>(false);
let retryTimeout: number = 0;

export const isConnected = () => {
	return connected;
};

export const connect = (address: string) => {
	ros.value = new ROSLIB.Ros({
		url: `ws://${address}:9090`
	});

	ros.value.on('connection', () => {
		console.log('Connected to websocket server.');
		connected = true;
	});

	ros.value.on('error', (error) => {
		console.log('Error connecting to websocket server: ', error);
	});

	ros.value.on('close', () => {
		console.log('Connection to websocket server closed.');
		// TODO: retry after a time out that increases exponentially
		connected = false;
	});
};
