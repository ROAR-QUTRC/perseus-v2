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
					value: ''
				}
			}
		}
	});

	export interface CameraEventType {
		type: 'camera';
		action:
			| 'group-description'
			| 'kill'
			| 'request-groups'
			| 'request-stream'
			| 'producer-ready'
			| 'group-terminated';
		data: {
			ip?: string;
			groupName?: string;
			// cameraName?: string;
			resolution?: { width: number; height: number };
			device?: string;
			cameras?: string[];
		};
	}
</script>

<script lang="ts">
	import WebrtcClient from '$lib/widgets/videoWebrtc/webrtcClient.svelte';
	import { io, type Socket } from 'socket.io-client';
	import { onMount, untrack } from 'svelte';
	import { toast } from 'svelte-sonner';

	let socket: Socket = io();

	// keep track of connected camera servers
	let devices = $state<
		{
			groupName: string;
			ip: string;
			cameras?: string[];
		}[]
	>([]);
	// keep track of the active devices for this widget
	let configObject = $state<Record<string, { deviceIp: string; cameras: string[] }>>({});
	let cameraNameMap = $state<Record<string, string>>({});

	socket.on('camera-event', (event: CameraEventType) => {
		// console.log(event);
		switch (event.action) {
			case 'group-description':
				if (devices.find((device) => device.groupName === event.data.groupName) === undefined) {
					// Add new device if it doesn't exist
					devices.push({
						ip: event.data.ip!,
						groupName: event.data.groupName!,
						cameras: event.data.cameras
					});
					if (configObject[event.data.groupName!] !== undefined) {
						configObject[event.data.groupName!].deviceIp = event.data.ip!;
					}

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

					// Tell server to start streams that are registered to that group
					for (const camera of settings.groups.cameraSetup.config.value!.split('\n')) {
						const [group, name, device] = camera.split(',');
						if (group === event.data.groupName) {
							socket.send({
								type: 'camera',
								action: 'request-stream',
								data: {
									groupName: group,
									device: device
								}
							} as CameraEventType);
						}
					}
				} else {
					console.error('Device already exists');
					toast.error('Device already exists');
				}
				break;
			case 'group-terminated':
				// remove the group when it goes offline
				let index = devices.findIndex((device) => device.groupName === event.data.groupName);
				devices.splice(index, 1);
				index = settings.groups.cameraSetup.group.options!.indexOf({
					value: event.data.groupName!,
					label: event.data.groupName!
				});
				settings.groups.cameraSetup.group.options!.splice(index, 1);
				configObject[event.data.groupName!].deviceIp = '';
				break;
			case 'request-groups':
			case 'request-stream':
				//ignore self sent request
				break;
			default:
				console.log(event);
		}
	});

	// Switch the options when the selected device group changes
	$effect(() => {
		if (settings.groups.cameraSetup.group.value === undefined) return;
		let group = settings.groups.cameraSetup.group.value;
		let cameras = devices.find((device) => device.groupName === group)?.cameras;
		settings.groups.cameraSetup.device.options = cameras?.map((camera) => {
			return {
				value: camera,
				label: camera.replace('-video-index0', '').replace('usb-', '').replaceAll('_', ' ')
			};
		});
	});

	$effect(() => {
		// ensure config has been applied
		let config: string | string[] | undefined = settings.groups.cameraSetup.config.value;
		if (config === undefined || config === '') return;
		config = config.split('\n').filter((camera) => camera !== '');
		untrack(() => {
			config.forEach((camera) => {
				const [group, name, device] = camera.split(',');
				let ip = devices.find((device) => device.groupName === group)?.ip;
				if (configObject[group] === undefined)
					configObject[group] = { deviceIp: ip ? ip : '', cameras: [] };
				if (!configObject[group].cameras.includes(device)) {
					configObject[group].cameras.push(device);
					cameraNameMap[device] = name;
				}

				settings.groups[group + ' ' + name] = {
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
							let resolution = settings.groups[group + ' ' + name].resolution.value!.split('x');
							socket.send({
								type: 'camera',
								action: 'request-stream',
								data: {
									groupName: group,
									device: device,
									resolution: { width: Number(resolution[0]), height: Number(resolution[1]) }
								}
							} as CameraEventType);
							return 'Told camera to resize';
						}
					},
					delete: {
						type: 'button',
						description: 'Delete camera stream',
						action: () => {
							// Tell camera server to stop stream
							// socket.send({
							// 	type: 'camera',
							// 	action: 'kill',
							// 	data: { groupName: group, device: device }
							// } as CameraEventType);

							configObject[group].cameras = configObject[group].cameras.filter(
								(camera) => camera !== device
							);
							if (cameraNameMap[device] !== undefined) {
								delete cameraNameMap[device];
							}
							if (configObject[group].cameras.length === 0) {
								delete configObject[group];
							}

							// Remove camera from config
							settings.groups.cameraSetup.config.value =
								settings.groups.cameraSetup.config.value!.replace(camera + '\n', '');
							delete settings.groups[group + ' ' + name];

							return 'Deleted camera stream';
						}
					}
				};
			});
		});
	});

	onMount(() => {
		settings.groups.cameraSetup.create.action = (): string => {
			if (
				settings.groups.cameraSetup.group.value === undefined ||
				settings.groups.cameraSetup.name.value === undefined ||
				settings.groups.cameraSetup.device.value === undefined
			) {
				return 'Please fill out all fields';
			}
			settings.groups.cameraSetup.config.value +=
				settings.groups.cameraSetup.group.value +
				',' +
				settings.groups.cameraSetup.name.value +
				',' +
				settings.groups.cameraSetup.device.value +
				'\n';

			socket.send({
				type: 'camera',
				action: 'request-stream',
				data: {
					groupName: settings.groups.cameraSetup.group.value,
					device: settings.groups.cameraSetup.device.value
				}
			} as CameraEventType);

			settings.groups.cameraSetup.group.value = undefined;
			settings.groups.cameraSetup.name.value = undefined;
			settings.groups.cameraSetup.device.value = undefined;

			return 'Created new camera';
		};
		settings.groups.cameraSetup.group.options = [];
		settings.groups.cameraSetup.group.value = undefined;
		settings.groups.cameraSetup.name.value = undefined;
		settings.groups.cameraSetup.device.value = undefined;

		// request video group data
		socket.send({ type: 'camera', action: 'request-groups' });

		return () => {
			socket.close();
		};
	});
</script>

{#each Object.keys(configObject) as device}
	{#if configObject[device].deviceIp !== ''}
		<WebrtcClient
			ip={configObject[device].deviceIp}
			groupName={device}
			cameras={configObject[device].cameras}
			cameraNames={cameraNameMap}
		/>
	{:else}
		<p>Waiting for {device} to come online...</p>
	{/if}
{:else}
	<p>
		No active streams. Check that peripheral servers are running and that you are on the correct
		network.
	</p>
{/each}
