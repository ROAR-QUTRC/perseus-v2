<script lang="ts">
	import Button from '$lib/components/ui/button/button.svelte';
	import { toggleMode, mode } from 'mode-watcher';
	import { CardStackPlus, Check, Moon, Sun, DotsHorizontal, OpenInNewWindow } from 'svelte-radix';
	import {
		activeWidgets,
		availableWidgets,
		getWidgetsByLayoutId,
		layouts,
		type WidgetType
	} from '$lib/scripts/state.svelte';
	import { repo } from 'remult';
	import * as DropdownMenu from '$lib/components/ui/dropdown-menu/index';
	import { Layout } from '../shared/Layout';
	import { onDestroy, onMount } from 'svelte';
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
		// Remove widgets that do not exist anymore
		activeLayout.widgets = activeLayout.widgets.filter((widget) => {
			return availableWidgets.some((availableWidget) => {
				return availableWidget.name === widget.name;
			});
		});
		// Check if widget is already added						<- maybe remove if this is desired behavior
		activeLayout.widgets.forEach((widget) => {
			if (widget.name === label) throw new Error(`Widget already added: ${label}`);
		});
		// Find a location for the new widget
		let x = 0;
		let y = 0;
		let foundPlace = false;
		while (!foundPlace) {
			// Check all the active widgets to see if the target space is available
			let clearedAllWidgets = true;
			activeLayout.widgets.forEach((widget) => {
				// check current position is in the bound of this weidget
				if (x >= widget.x && x < widget.x + widget.w && y >= widget.y && y < widget.y + widget.h) {
					clearedAllWidgets = false;
				}
			});
			// if it is then create the widget
			if (clearedAllWidgets) {
				foundPlace = true;
			} else if (x == 9 && y == 9) {
				// if it isnt and the last position is reached then throw an error
				foundPlace = true;
				throw new Error('No more space for widgets');
			} else if (x == 9) {
				// Move to next row
				x = 0;
				y++;
			} else {
				// Move to next column
				x++;
			}
		}
		// Add widget
		repo(Layout).update(activeLayout.id, {
			widgets: [
				...activeLayout.widgets,
				{
					name: label,
					x: x,
					y: y,
					w: 1,
					h: 1
				}
			]
		});
	};

	const removeWidget = (name: string) => {
		repo(Layout).update(layouts.value[layouts.active].id, {
			widgets: layouts.value[layouts.active].widgets.filter((widget) => widget.name !== name)
		});
	};

	let isMobile = $state<boolean>(false);

	onMount(() => {
		// check if the device is mobile
		if (/Mobi|Android|iPhone|iPad|iPod|BlackBerry|Windows Phone/i.test(navigator.userAgent)) {
			isMobile = true;
		}
		console.log('isMobile:', navigator);
	});

	let openSettings = $state(false);
	let openWidgetAddMenu = $state(false);
</script>

<div class="h-full flex-col">
	<!-- Menu bar -->
	<div class="flex border-b p-2">
		<LayoutMenu />

		<ConnectionMenu {isMobile} />

		{#if isMobile}
			<DropdownMenu.Root>
				<DropdownMenu.Trigger>
					<Button variant="outline" size="icon"><DotsHorizontal /></Button>
				</DropdownMenu.Trigger>
				<DropdownMenu.Content align="end">
					<DropdownMenu.Item onclick={() => (openWidgetAddMenu = true)}>
						Add Widget <OpenInNewWindow class="ml-auto h-4 w-4" />
					</DropdownMenu.Item>
					<DropdownMenu.Item onclick={() => (openSettings = true)}>
						Settings <OpenInNewWindow class="ml-auto h-4 w-4" />
					</DropdownMenu.Item>
					<DropdownMenu.Item onclick={toggleMode}>
						{#if $mode === 'light'}
							Dark Mode<Moon class="ml-auto h-4 w-4" />
						{:else}
							Light Mode<Sun class="ml-auto h-4 w-4" />
						{/if}
					</DropdownMenu.Item>
				</DropdownMenu.Content>
			</DropdownMenu.Root>
			<Dialog.Root bind:open={openWidgetAddMenu}>
				<Dialog.Content class="w-[80vw]">
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
				</Dialog.Content>
			</Dialog.Root>
		{:else}
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
				<Button variant="outline" size="icon" onclick={() => (openSettings = true)}>
					<Gear />
				</Button>

				<!-- Toggle light and dark mode -->
				<Button onclick={toggleMode} variant="outline" size="icon">
					{#if $mode === 'light'}
						<Moon />
					{:else}
						<Sun />
					{/if}
				</Button>
			</div>
		{/if}
	</div>

	<!-- Widget canvas -->
	<WidgetCanvas />
</div>
<Dialog.Root bind:open={openSettings}>
	<Dialog.Content class="w-[80vw]">
		<Dialog.Header>
			<Dialog.Title>Layout Settings</Dialog.Title>
		</Dialog.Header>
		<SettingsForm layout={activeLayout} />
	</Dialog.Content>
</Dialog.Root>
