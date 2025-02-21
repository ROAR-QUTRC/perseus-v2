<script lang="ts">
	import Button from '$lib/components/ui/button/button.svelte';
	import { toggleMode, mode } from 'mode-watcher';
	import { CardStackPlus, Check, Moon, Sun } from 'svelte-radix';
	import {
		activeWidgets,
		availableWidgets,
		getWidgetsByLayoutId,
		layouts,
		type WidgetType
	} from '$lib/scripts/state.svelte';
	import { repo } from 'remult';
	import { Layout } from '../shared/Layout';
	import { onDestroy } from 'svelte';
	import LayoutMenu from '$lib/components/layout-menu.svelte';
	import * as Dialog from '$lib/components/ui/dialog/index';
	import { Gear } from 'svelte-radix';
	import SettingsForm from '$lib/components/settings-form.svelte';
	import * as Command from '$lib/components/ui/command/index';
	import * as Popover from '$lib/components/ui/popover/index';
	import WidgetCanvas from '$lib/components/widget-canvas.svelte';
	import ConnectionMenu from '$lib/components/connection-menu.svelte';

	let unSub: (() => void) | null = null;
	let widgetGroups = $state<Array<Array<WidgetType>>>([]);

	widgetGroups = Object.values(
		availableWidgets.reduce((acc: any, item) => {
			// If group is not specified put it in the Misc group
			if (!item.group) item.group = 'Misc';
			// Append the item to the array for each group
			acc[item.group] = [...(acc[item.group] || []), item];
			return acc;
		}, {})
	);

	$inspect(widgetGroups);

	$effect(() => {
		unSub = repo(Layout)
			.liveQuery()
			.subscribe((l) => {
				layouts.value = l.applyChanges(layouts.value);
				// Load the saved state of the widgets when the db changes
				if (layouts.value.length === 0) {
					repo(Layout).save({
						title: 'Default',
						widgets: []
					});
				} else activeWidgets.value = getWidgetsByLayoutId(layouts.value[layouts.active].id);
			});
	});
	const activeLayout = $derived<Layout>(layouts.value[layouts.active]);
	const widgetList = $derived.by<Array<string>>(() => {
		if (activeLayout) return activeLayout.widgets.map((widget) => widget.name);
		return [];
	});

	onDestroy(() => {
		unSub && unSub();
	});

	// Check for bad components
	$effect(() => {
		let indexedWidgets: Array<string> = [];
		availableWidgets.forEach((widget) => {
			if (indexedWidgets.includes(widget.name))
				throw new Error(`Loaded duplicate widget label: ${widget.name}`);
			indexedWidgets.push(widget.name);
		});
	});

	const addWidget = (label: string) => {
		// Check if widget is already added						<- maybe remove if this is desired behavior
		activeLayout.widgets.forEach((widget) => {
			if (widget.name === label) throw new Error(`Widget already added: ${label}`);
		});
		// Add widget
		repo(Layout).update(activeLayout.id, {
			widgets: [
				...activeLayout.widgets,
				{
					name: label,
					x: 0,
					y: 0,
					w: 2,
					h: 2
				}
			]
		});
	};

	const removeWidget = (name: string) => {
		repo(Layout).update(layouts.value[layouts.active].id, {
			widgets: layouts.value[layouts.active].widgets.filter((widget) => widget.name !== name)
		});
	};
</script>

<div class="h-full flex-col">
	<!-- Menu bar -->
	<div class="flex border-b p-2">
		<LayoutMenu />

		<ConnectionMenu />

		<div class="flex space-x-2">
			<!-- Add widget -->
			<Popover.Root>
				<Popover.Trigger>
					<Button variant="outline" size="icon">
						<CardStackPlus />
					</Button>
				</Popover.Trigger>
				<Popover.Content class="mx-2 max-h-[600px] w-[200px] p-0">
					<Command.Root>
						<Command.Input placeholder="Search widgets..." />
						<Command.List>
							<Command.Empty>No widgets found.</Command.Empty>
							{#each widgetGroups as group}
								<Command.Group heading={group[0].group}>
									{#each group as widget}
										<Command.Item
											onclick={() => {
												widgetList.includes(widget.name)
													? removeWidget(widget.name)
													: addWidget(widget.name);
											}}
										>
											{widget.name}
											{#if widgetList.includes(widget.name)}
												<Check class="ml-auto h-4 w-4" />
											{/if}
										</Command.Item>
									{/each}
								</Command.Group>
							{/each}
						</Command.List>
					</Command.Root>
				</Popover.Content>
			</Popover.Root>
			<!-- Settings form -->
			<Dialog.Root>
				<Dialog.Trigger>
					<Button variant="outline" size="icon">
						<Gear />
					</Button>
				</Dialog.Trigger>
				<Dialog.Content class="w-[80vw]">
					<Dialog.Header>
						<Dialog.Title>Layout Settings</Dialog.Title>
					</Dialog.Header>
					<SettingsForm layout={activeLayout} />
				</Dialog.Content>
			</Dialog.Root>
			<!-- Toggle light and dark mode -->
			<Button onclick={toggleMode} variant="outline" size="icon">
				{#if $mode === 'dark'}
					<Moon />
				{:else}
					<Sun />
				{/if}
			</Button>
		</div>
	</div>

	<!-- Widget canvas -->
	<WidgetCanvas />
</div>
