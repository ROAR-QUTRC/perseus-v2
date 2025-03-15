<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Video canvas - WebRTC';
	export const description =
		"For viewing multiple video streams that are being handled by the video servers run with 'yarn camera'";
	export const group = 'Gstreamer';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			cameraSetup: {
				group: {
					type: 'select',
					description: 'Name of the camera group',
					options: []
				},
				name: {
					type: 'text',
					description: 'Name of the camera in the group'
				},
				device: {
					type: 'text',
					description:
						'Device that video is streamed from. Use the linux device name (e.g. /dev/video0) or "test"'
				},
				create: {
					type: 'button',
					description: 'Create a new camera stream'
				},
				config: {
					type: 'text',
					disabled: true,
					value: ''
				}
			}
		}
	});

	export interface CameraEventType {
		type: 'camera';
		action: 'group-description' | 'kill' | 'request-groups' | 'request-stream' | 'producer-ready';
		data: {
			ip?: string;
			groupName?: string;
			cameraName?: string;
			resolution?: { width: number; height: number };
			device?: string;
		};
	}
</script>

<script lang="ts">
	import WebrtcClient from '$lib/widgets/videoWebrtc/webrtcClient.svelte';
	import { io, type Socket } from 'socket.io-client';
	import { onMount } from 'svelte';
	import { toast } from 'svelte-sonner';

	let socket: Socket = io();

	let devices = $state<
		{
			groupName: string;
			ip: string;
			port: number;
		}[]
	>([]);
	// $inspect(devices);

	socket.on('camera-event', (event: CameraEventType) => {
		// console.log(event);
		switch (event.action) {
			case 'group-description':
				if (devices.find((device) => device.groupName === event.data.groupName) === undefined) {
					devices.push({
						ip: event.data.ip!,
						port: 8443,
						groupName: event.data.groupName!
					});

					// Add group to the settings
					if (
						!settings.groups.cameraSetup.group.options?.includes({
							value: event.data.groupName!,
							label: event.data.groupName!
						})
					) {
						settings.groups.cameraSetup.group.options?.push({
							value: event.data.groupName!,
							label: event.data.groupName!
						});
					}

					// Tell server to start streams
					for (const camera of settings.groups.cameraSetup.config.value!.split(',')) {
						const [group, name, device] = camera.split('.');
						if (group === event.data.groupName) {
							socket.send({
								type: 'camera',
								action: 'request-stream',
								data: {
									groupName: group,
									cameraName: name,
									device: device,
									resolution: { width: 320, height: 240 }
								}
							} as CameraEventType);
						}
					}
				} else {
					console.error('Device already exists');
					toast.error('Device already exists');
				}
				break;
			case 'kill':
				const index = devices.findIndex((device) => device.groupName === event.data.groupName);
				devices.splice(index, 1);
				break;
			case 'request-groups':
			case 'request-stream':
				//ignore self sent request
				break;
			default:
				console.log(event);
		}
	});

	$effect(() => {
		// ensure config has been applied
		let config: string | string[] | undefined = settings.groups.cameraSetup.config.value;
		if (config === undefined || config === '') return;
		config = config.split(',').filter((camera) => camera !== '');
		config.forEach((camera) => {
			const [group, name, device] = camera.split('.');
			settings.groups[group + ': ' + name] = {
				resolution: {
					type: 'select',
					description: 'Resolution of the camera',
					options: [
						{ value: '1920x1080', label: '1920x1080' },
						{ value: '1280x720', label: '1280x720' },
						{ value: '640x480', label: '640x480' },
						{ value: '320x240', label: '320x240' }
					],
					value: '320x240'
				},
				device: {
					type: 'text',
					disabled: true,
					value: device
				},
				restartCamera: {
					type: 'button',
					description: 'Restart camera stream',
					action: () => {
						return 'TODO';
					}
				},
				stop: {
					type: 'button',
					description: 'Stop camera stream',
					action: () => {
						// socket.send({ type: 'camera', action: 'request-stream', data: { groupName: group } });
						// return 'Killed camera stream';
						return 'TODO';
					}
				}
			};
		});
	});

	onMount(() => {
		settings.groups.cameraSetup.create.action = (): string => {
			settings.groups.cameraSetup.config.value +=
				settings.groups.cameraSetup.group.value +
				'.' +
				settings.groups.cameraSetup.name.value +
				'.' +
				settings.groups.cameraSetup.device.value +
				',';

			return 'Created new camera';
		};

		// request video group data
		socket.send({ type: 'camera', action: 'request-groups' });

		return () => {
			socket.close();
		};
	});
</script>

{#each devices as device}
	<WebrtcClient ip={device.ip} port={device.port} groupName={device.groupName} />
{:else}
	<p>
		No active streams. Check that peripheral servers are running and that you are on the correct
		network.
	</p>
{/each}
