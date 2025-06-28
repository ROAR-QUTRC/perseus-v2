<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'WebRTC Video';
	export const description =
		"For viewing multiple video streams that are being handled by the video servers run with 'yarn camera'";
	export const group = 'Gstreamer';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			setupCamera: {
				name: {
					type: 'text',
					description: 'Name of the camera in the group'
				},
				device: {
					type: 'select',
					description: 'Device that video is streamed from.',
					options: []
				},
				create: {
					type: 'button',
					description: 'Create a new camera stream'
				},
				config: {
					type: 'text',
					disabled: true,
					value: '{}'
				}
			}
		}
	});

	export type videoTransformType =
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

	export interface ConfigType {
		name: string;
		resolution: {
			width: number;
			height: number;
		};
		transform: videoTransformType;
	}
</script>

<script lang="ts">
	import { onMount, untrack } from 'svelte';
	import { ScrollArea } from '$lib/components/ui/scroll-area/index';
	import { io, type Socket } from 'socket.io-client';
	import {
		connectToSignallingServer,
		getPeerId,
		newPeerConfigured,
		peerConnections,
		ws
	} from './webrtc/signalHandler.svelte';
	import VideoWrapper from './webrtc/videoWrapper.svelte';
	import WebrtcStream from './videoWebrtc-old/webrtcStream.svelte';

	let socket: Socket = io();

	let devices = $state<string[]>([]);
	let config = $derived<Record<string, ConfigType>>(
		JSON.parse(settings.groups.setupCamera.config.value!) || {}
	);

	// Helper functions
	const formatDeviceName = (device: string): string =>
		// @ts-ignore
		device.replace('-video-index0', '').replace('usb-', '').replaceAll('_', ' ');

	const updateAvailableDevices = (device: string, addingNewDevice: boolean) => {
		if (addingNewDevice && !devices.includes(device)) {
			devices.push(device);
			if (!settings.groups.setupCamera.device.options)
				settings.groups.setupCamera.device.options = [];
			settings.groups.setupCamera.device.options.push({
				value: device,
				label: formatDeviceName(device)
			});
		} else if (!addingNewDevice && devices.includes(device)) {
			devices = devices.filter((d) => d !== device);
			if (settings.groups.setupCamera.device.options) {
				settings.groups.setupCamera.device.options =
					settings.groups.setupCamera.device.options.filter((option) => option.value !== device);
			}
		}
	};

	// Ensure the peer connection list contains all the configured devices
	$effect(() => {
		// update the peer connections list with the current config
		const devices = Object.keys(config);
		untrack(() => {
			devices.forEach((device) => {
				if (!peerConnections[device]) {
					peerConnections[device] = {
						sessionId: '',
						name: config[device].name,
						online: false,
						connection: null,
						track: null
					};
				}
			});
			// Remove any peer connections that are not in the config
			Object.keys(peerConnections).forEach((device) => {
				if (!devices.includes(device)) {
					peerConnections[device].connection?.close();
					peerConnections[device].track = null;
					delete peerConnections[device];
				}
			});
		});
	});

	// Handle incoming camera events
	socket.on('camera-event', (event: CameraEventType) => {
		switch (event.action) {
			case 'group-description':
				event.data!.devices?.forEach((device) => {
					updateAvailableDevices(device, true);
				});

				// request streams for cameras in config
				Object.keys(config).forEach((device) => {
					if (event.data.devices?.includes(device)) {
						socket.send({
							type: 'camera',
							action: 'request-stream',
							data: {
								devices: [device],
								resolution: config[device].resolution,
								transform: config[device].transform
							}
						} as CameraEventType);
					}
				});
				break;
			case 'device-disconnect':
				// Remove device from the list
				updateAvailableDevices(event.data.devices![0], false);
				break;
			case 'group-terminated':
				// remove all peer connections for this group
				event.data.devices?.forEach((device) => {
					if (peerConnections[device]) {
						peerConnections[device].connection?.close();
						peerConnections[device] = {
							sessionId: '',
							name: peerConnections[device].name,
							online: false,
							connection: null,
							track: null
						};
					}
				});
				// Remove device from the list
				updateAvailableDevices(event.data.devices![0], false);
				break;
			case 'kill':
				break;
			// Ignore self sent events
			case 'request-groups':
			case 'request-stream':
				break;
			default:
				console.warn('Unknown camera event action:', event.action);
		}
	});

	onMount(() => {
		// Add button actions
		settings.groups.setupCamera.device.options = [];
		settings.groups.setupCamera.create.action = (): string => {
			const values = settings.groups.setupCamera;
			if (!values.name.value || !values.device.value) {
				return 'Please fill in all fields';
			}

			if (config[values.device.value]) {
				return 'Camera with this name already exists';
			}

			config[values.device.value] = {
				name: values.name.value,
				resolution: { width: 320, height: 240 }, // Default resolution
				transform: 'none' // Default transform
			};

			// Update settings config field
			settings.groups.setupCamera.config.value = JSON.stringify(config);

			// Send request to create camera
			socket.send({
				type: 'camera',
				action: 'request-stream',
				data: {
					devices: [values.device.value],
					resolution: config[values.device.value].resolution,
					transform: config[values.device.value].transform
				}
			} as CameraEventType);

			// Reset the form
			values.name.value = '';
			values.device.value = '';

			newPeerConfigured();

			return 'Created new camera';
		};

		// send initial request for camera groups
		socket.send({ type: 'camera', action: 'request-groups' } as CameraEventType);

		connectToSignallingServer(window.location.hostname);
		return () => {
			// Web server socket
			socket.close();
			// signalling server socket
			ws?.close();
			// Close all peer connections
			Object.keys(peerConnections).forEach((key) => {
				peerConnections[key].connection?.close();
				peerConnections[key].track = null;
				delete peerConnections[key];
			});
		};
	});

	$inspect(peerConnections);
	$inspect(config);

	// -------------------------------------
	// Video settings functions
	// -------------------------------------

	const onVideoClose = (device: string) => {
		if (peerConnections[device]) {
			peerConnections[device].connection?.close();
			peerConnections[device].track = null;
			peerConnections[device].online = false;
			delete peerConnections[device];
		}

		// Remove from config
		if (config[device]) {
			delete config[device];
			settings.groups.setupCamera.config.value = JSON.stringify(config);
		}
	};

	const onVideoSettingsChange = (device: string, newConfig: ConfigType) => {
		// Update the config
		config[device] = newConfig;
		settings.groups.setupCamera.config.value = JSON.stringify(config);

		// Send request to update stream
		socket.send({
			type: 'camera',
			action: 'request-stream',
			data: {
				devices: [device],
				resolution: newConfig.resolution,
				transform: newConfig.transform
			}
		} as CameraEventType);
	};

	const onVideoRestart = (device: string) => {
		socket.send({
			type: 'camera',
			action: 'request-stream',
			data: {
				devices: [device],
				resolution: config[device].resolution,
				transform: config[device].transform,
				forceRestart: true
			}
		} as CameraEventType);
	};
</script>

<ScrollArea orientation="vertical" class="relative flex h-full w-full">
	<p class="bg-card absolute bottom-1 left-1 rounded-[4px] bg-opacity-60 px-2 py-1">
		Session ID: {getPeerId()}
	</p>
	{#if Object.keys(peerConnections).length === 0}
		<div
			class="absolute left-1/2 top-1/2 flex -translate-x-1/2 -translate-y-1/2 flex-col items-center justify-center"
		>
			<p class="text-center">No video streams available.</p>
			<p class="text-center">Please create a new camera stream in the settings.</p>
		</div>
	{/if}
	<div class="flex flex-row flex-wrap">
		{#each Object.keys(peerConnections) as peer}
			<div class="relative m-2 min-h-[160px] min-w-[240px] overflow-hidden rounded-lg border">
				<VideoWrapper
					device={peer}
					config={config[peer]}
					{onVideoClose}
					{onVideoSettingsChange}
					{onVideoRestart}
				/>
			</div>
		{/each}
	</div>
</ScrollArea>
