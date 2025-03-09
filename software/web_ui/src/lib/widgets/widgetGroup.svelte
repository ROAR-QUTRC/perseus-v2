<script lang="ts" module>
	import Widget from '$lib/components/widget.svelte';
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import {
		availableWidgets,
		type WidgetSettingsType,
		type WidgetType
	} from '$lib/scripts/state.svelte';

	export const name = 'Widget Group';
	export const description = 'Used to group multiple widgets together';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	import * as DropdownMenu from '$lib/components/ui/dropdown-menu';
	import Grid, { GridItem } from 'svelte-grid-extended';
	import Button from '$lib/components/ui/button/button.svelte';

	let widgets = $state<WidgetType[]>([]);

	const addWidget = (widget: WidgetType) => {
		widgets.push(widget);
	};
</script>

<div class="flex h-full flex-col">
	<div class="-mx-2 flex flex-row border-b px-2 pb-2">
		<p class="my-auto">Sub widgets: {widgets.length}</p>
		<DropdownMenu.Root>
			<DropdownMenu.Trigger class="ml-auto">
				<Button variant="outline" size="sm">Add Widget</Button>
			</DropdownMenu.Trigger>
			<DropdownMenu.Content>
				{#each availableWidgets as widget}
					<DropdownMenu.Item onclick={() => addWidget(widget)}>
						{widget.name}
					</DropdownMenu.Item>
				{/each}
			</DropdownMenu.Content>
		</DropdownMenu.Root>
	</div>
	<Grid gap={5} cols={20} rows={20}>
		{#each widgets as widget, i}
			<GridItem id={widget.name} x={i} y={0}></GridItem>
		{/each}
	</Grid>
</div>
