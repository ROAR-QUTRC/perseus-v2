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
			},
			removeCamera: {
				name: {
					type: 'select',
					description: 'Name of the camera in the group',
					options: []
				},
				remove: {
					type: 'button',
					description: 'Remove a camera stream'
				}
			}
		}
	});

	type videoTransformType =
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
		};
	}
</script>

<script lang="ts">
	import { onMount, untrack } from 'svelte';
	import { ScrollArea } from '$lib/components/ui/scroll-area/index';
	import { io, type Socket } from 'socket.io-client';
	import {
		connectToSignallingServer,
		getPeerId,
		peerConnections,
		ws
	} from './webrtc/signalHandler.svelte';
	import VideoWrapper from './webrtc/videoWrapper.svelte';

	let socket: Socket = io();

	let devices = $state<string[]>([]);
	let config = $derived<
		Record<
			string,
			{
				name: string;
				resolution: {
					width: number;
					height: number;
				};
				transform: videoTransformType;
			}
		>
	>(JSON.parse(settings.groups.setupCamera.config.value!) || {});

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

	const formatDeviceName = (device: string): string =>
		// @ts-ignore
		device.replace('-video-index0', '').replace('usb-', '').replaceAll('_', ' ');

	socket.on('camera-event', (event: CameraEventType) => {
		switch (event.action) {
			case 'group-description':
				event.data!.devices?.forEach((device) => {
					if (!devices.includes(device)) {
						devices.push(device);
						if (!settings.groups.setupCamera.device.options)
							settings.groups.setupCamera.device.options = [];
						settings.groups.setupCamera.device.options.push({
							value: device,
							label: formatDeviceName(device)
						});
					}
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
				devices = devices.filter((device) => device !== event.data!.devices![0]);
				if (settings.groups.setupCamera.device.options) {
					settings.groups.setupCamera.device.options =
						settings.groups.setupCamera.device.options.filter(
							(option) => option.value !== event.data!.devices![0]
						);
				}
				break;
			case 'group-terminated':
				// remove all peer connections for this group
				//TODO:
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
					devices: [values.device.value]
				}
			} as CameraEventType);

			// Reset the form
			values.name.value = '';
			values.device.value = '';

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
</script>

<ScrollArea orientation="vertical" class="flex h-full w-full">
	<p>{getPeerId()}</p>
	<div class="flex flex-row flex-wrap">
		{#each Object.keys(peerConnections) as peer}
			<div class="relative m-2 grid h-[160px] w-[240px] place-content-center rounded-lg border">
				{#if peerConnections[peer].track}
					<VideoWrapper media={peerConnections[peer].track} name={peerConnections[peer].name} />
				{:else if peerConnections[peer].online}
					<p>Waiting for {peerConnections[peer].name} to come online...</p>
				{:else}
					<p class="text-center">Connecting to {peerConnections[peer].name}...</p>
				{/if}
			</div>
		{/each}
		<!-- On screen create stream button -->
		<!-- <div
			class="relative m-2 grid h-[160px] w-[240px] cursor-pointer place-content-center rounded-lg border"
		>
			<div
				class="grid h-[50px] w-[50px] grid-cols-2 place-content-center rounded-[50%] border-[3px] px-[7px]"
			>
				<span class="m-0 h-[15px] w-[15px] border-b-[2px] border-r-[2px] p-0"></span>
				<span class="m-0 h-[15px] w-[15px] border-b-[2px] border-l-[2px] p-0"></span>
				<span class="m-0 h-[15px] w-[15px] border-r-[2px] border-t-[2px] p-0"></span>
				<span class="m-0 h-[15px] w-[15px] border-l-[2px] border-t-[2px] p-0"></span>
			</div>
		</div> -->
	</div>
</ScrollArea>
