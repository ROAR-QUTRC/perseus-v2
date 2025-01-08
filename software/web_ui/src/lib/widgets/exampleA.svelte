<script lang="ts" module>
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Example A';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			General: {
				FirstSetting: {
					type: 'text',
					value: 'Example'
				},
				SecondSetting: {
					type: 'text',
					description: 'This is a description'
				}
			}
		}
	});
</script>

<script lang="ts">
</script>

<p>{settings.groups.General.FirstSetting.value ?? 'No data'}</p>
