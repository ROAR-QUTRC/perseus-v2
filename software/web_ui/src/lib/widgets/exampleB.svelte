<script lang="ts" module>
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Example B';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			general: {
				textSetting: {
					type: 'text',
					value: 'Some text'
				},
				hereIsAButton: {
					type: 'button',
					action: () => {
						console.log('Button clicked');
						return 'Success';
					}
				}
			},
			Advanced: {
				booleanSwitch: {
					type: 'switch',
					description: 'This is a switch'
				},
				selectOptions: {
					type: 'select',
					options: [
						{ value: '1', label: 'Option 1' },
						{ value: '2', label: 'Option 2' },
						{ value: '3', label: 'Option 3' }
					]
				}
			}
		}
	});
</script>

<script lang="ts">
</script>

<table style="border-collapse: collapse;border: 2px solid rgb(140 140 140);">
	<thead>
		<tr>
			<th>Setting</th>
			<th>Value</th>
		</tr>
	</thead>
	<tbody>
		<tr>
			<td>text</td>
			<td>{settings.groups.general.textSetting.value}</td>
		</tr>
		<tr>
			<td>number</td>
			<td>{settings.groups.general.hereIsAButton.value}</td>
		</tr>
		<tr>
			<td>switch</td>
			<td>{settings.groups.Advanced.booleanSwitch.value}</td>
		</tr>
		<tr>
			<td>options</td>
			<td>{settings.groups.Advanced.selectOptions.value}</td>
		</tr>
	</tbody>
</table>
