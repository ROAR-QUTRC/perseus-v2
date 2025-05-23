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
					value: ''
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
</script>
