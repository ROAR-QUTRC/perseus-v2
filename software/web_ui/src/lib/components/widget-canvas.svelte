<script lang="ts">
	import { activeWidgets, layouts } from '$lib/scripts/state.svelte';
	import Grid, { GridItem } from 'svelte-grid-extended';
	import Widget from './widget.svelte';
	import * as Card from '$lib/components/ui/card/index';
	import { CornerBottomRight } from 'svelte-radix';
	import { Layout } from '../../shared/Layout';
	import { cn } from '$lib/utils';
	import { repo } from 'remult';

	const activeLayout = $derived<Layout>(layouts.value[layouts.active]);

	// on:change event handler triggers twice lol
	let lastTimeStamp = -1;
	const updateWidgetPosition = (event: any, name: string) => {
		// if the difference in time stamps is less than 3ms, ignore the event
		if (!(event.timeStamp - lastTimeStamp < 3)) {
			lastTimeStamp = event.timeStamp;

			let widgetDraft: Partial<Layout> = { widgets: activeLayout.widgets };

			widgetDraft.widgets?.forEach((widget) => {
				if (widget.name === name) {
					widget.x = event.detail.item.x;
					widget.y = event.detail.item.y;
					widget.w = event.detail.item.w;
					widget.h = event.detail.item.h;
				}
			});

			repo(Layout).update(activeLayout.id, widgetDraft);
		}
	};
</script>

{#if layouts.value.length !== 0}
	<Grid cols={10} rows={10} gap={5}>
		{#each activeWidgets.value as widgetData}
			<GridItem
				id={widgetData.name}
				x={widgetData.layoutProps!.x}
				y={widgetData.layoutProps!.y}
				w={widgetData.layoutProps!.w}
				h={widgetData.layoutProps!.h}
				on:change={(e) => updateWidgetPosition(e, widgetData.name)}
			>
				<div class="h-[100%]" slot="moveHandle" let:moveStart>
					<Widget {widgetData}>
						{#snippet dragHandle()}
							<Card.Title
								class={cn('flex-1 overflow-hidden text-ellipsis text-nowrap p-2', {
									'hover:cursor-move': !activeLayout.isLocked
								})}
								onpointerdown={activeLayout.isLocked ? () => {} : moveStart}
							>
								{widgetData.name}
							</Card.Title>
						{/snippet}
					</Widget>
				</div>
				<div
					class="absolute bottom-0 right-0 m-1 hover:cursor-nwse-resize"
					slot="resizeHandle"
					let:resizeStart
				>
					{#if !activeLayout.isLocked}
						<CornerBottomRight onpointerdown={resizeStart} />
					{/if}
				</div>
			</GridItem>
		{/each}
	</Grid>
{/if}
