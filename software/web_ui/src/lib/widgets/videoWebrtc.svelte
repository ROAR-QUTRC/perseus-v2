<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.

	// gst-launch-1.0 webrtcsink run-signalling-server=true stun-server=NULL name=ws v4l2src device=/dev/video0 ! video/x-raw, width=1024, height=576 ! videoconvert ! ws.

	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Video - WebRTC';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			registerCamera: {
				name: {
					type: 'text',
					description: 'Enter the name of the stream',
					value: 'Camera 1'
				},
				host: {
					type: 'text',
					description: 'Select the ip of the host that the stream is hosted on',
					options: [],
					value: '10.1.1.133'
				},
				port: {
					type: 'number',
					description: 'Enter the port of the host that the stream is hosted on',
					value: '8443'
				},
				StartSession: {
					type: 'button',
					description: 'Start a new session',
					action: () => {
						if (Object.keys(settings.groups).includes(settings.groups.registerCamera.name.value!)) {
							return 'Session already exists';
						}

						settings.groups.registerCamera.cameraConfig.value += `${settings.groups.registerCamera.name.value}@${
							settings.groups.registerCamera.host.value
						}:${settings.groups.registerCamera.port.value},`;

						return 'Created session';
					}
				},
				debugMode: {
					type: 'switch',
					description: 'Enable debug mode',
					value: 'false'
				},
				cameraConfig: {
					// cameraname@ip:port,cameraName@ip:port
					type: 'readonly',
					description: 'Read only, for internal use',
					value: ''
				}
			}
		}
	});
</script>

<script lang="ts">
	import { onMount } from 'svelte';
	import { WebRtcSession } from '$lib/widgets/videoWebrtc/webrtc';
	import VideoWrapper from '$lib/widgets/videoWebrtc/videoWrapper.svelte';
	import panzoom from 'panzoom';

	// ensure settings buttons have actions
	$effect(() => {
		let config: string | string[] | undefined = settings.groups.registerCamera.cameraConfig.value;
		if (config && config.length > 0) {
			console.log('configging');

			config = settings.groups.registerCamera.cameraConfig
				.value!.split(',')
				.filter((session) => session !== '');

			config.forEach((camera) => {
				let [name, host] = camera.split('@');
				let [ip, port] = host.split(':');

				// if no action is defined, add a close session action
				if (!settings.groups[name] || !settings.groups[name].closeSession?.action) {
					// Create settings field
					settings.groups[name] = {
						closeSession: {
							type: 'button',
							description: 'Close the session',
							action: () => {
								settings.groups.registerCamera.cameraConfig.value =
									settings.groups.registerCamera.cameraConfig.value!.replace(camera + ',', '');
								delete settings.groups[name];
								return 'Session closed';
							}
						}
					};

					// Create media source
					const i = mediaSources.push(new WebRtcSession(name, ip, port, true));
					mediaSources[i - 1].onNewTrack = (track, info) => {
						console.log(tracks.length);
						// if (!tracks.find((t) => t.info.clientId === info.clientId)) {
						if (tracks.length < 2) {
							tracks.push({ stream: track, info: info });
						}
					};
				}
			});

			// ensure connections to each source and close connected that no longer exist
			mediaSources.filter((source) => {
				if (!config?.includes(`${source.name}@${source.ip}:${source.port}`)) {
					if (source.signallingSocket) {
						source.signallingSocket.close();
					}
					return false;
				} else return true;
			});
		}
	});

	let mediaSources = $state<WebRtcSession[]>([]); // Connections made to devices
	let tracks = $state<
		{ stream: MediaStream; info: { clientId: string; producerId: string; sessionId: string } }[]
	>([]); // Tracks from the devices

	onMount(() => {
		return () => {
			mediaSources.forEach((source) => {
				if (source.signallingSocket) {
					source.signallingSocket.close();
				}
			});
		};
	});

	let instance;
	const initPanZoom = (node: HTMLElement) => {
		instance = panzoom(node, { maxZoom: 2, minZoom: 0.5 });
	};
</script>

<div class="h-full w-full">
	<div class="flex flex-row flex-wrap bg-orange-400" use:initPanZoom>
		{#each tracks as track, i}
			<div class=" m-1 bg-slate-500">
				<p>{i}</p>
				<p>Client id: {track.info.clientId}</p>
				<p>Producer id: {track.info.producerId}</p>
				<p>Session id: {track.info.sessionId}</p>
				<!-- svelte-ignore a11y_media_has_caption -->
				<VideoWrapper media={track.stream} />
			</div>
		{/each}
	</div>
</div>
