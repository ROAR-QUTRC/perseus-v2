<script lang="ts">
	import * as Popover from '$lib/components/ui/popover/index';
	import * as Dialog from '$lib/components/ui/dialog/index';
	import * as Command from '$lib/components/ui/command/index';
	import * as Collapsible from '$lib/components/ui/collapsible/index';
	import * as Card from '$lib/components/ui/card/index';
	import * as HoverCard from '$lib/components/ui/hover-card/index';
	import { Button } from '$lib/components/ui/button/index';
	import { ScrollArea } from '$lib/components/ui/scroll-area/index';
	import { CaretSort, Check, ChevronDown, ChevronUp, Pencil2, PlusCircled } from 'svelte-radix';
	import { activeWidgets, getWidgetsByLayoutId, layouts } from '$lib/scripts/state.svelte';
	import { cn } from '$lib/utils';
	import Label from '$lib/components/ui/label/label.svelte';
	import Input from '$lib/components/ui/input/input.svelte';
	import SettingsForm from './settingsForm.svelte';
	import { repo } from 'remult';
	import { Layout } from '../../shared/Layout';
	import { localStore } from '$lib/scripts/localStore.svelte';
	import { onMount } from 'svelte';

	let open = $state(false);
	let showLayoutDialog = $state<'closed' | 'manage' | 'create'>('closed');
	let createForm = $state<{ title: string; description: string }>({ title: '', description: '' });

	// for styling the layout manager dialog
	let openStates = $state<Array<boolean>>([]);
	function handleOpenChange(layoutIndex: number, isOpen: boolean) {
		openStates[layoutIndex] = isOpen;
	}
	$effect(() => {
		if (showLayoutDialog === 'closed') openStates = [];
	});

	const createLayout = () => {
		repo(Layout).insert({
			title: createForm.title,
			description: createForm.description,
			widgets: []
		});
		showLayoutDialog = 'closed';
	};

	let activeLayoutStore = localStore('activeLayout', 0);

	const changeLayout = (i: number) => {
		activeLayoutStore.value = i;
		layouts.active = i;
		activeWidgets.value = getWidgetsByLayoutId(layouts.value[layouts.active].id);
	};

	onMount(() => {
		layouts.active = activeLayoutStore.value;

		return () => {
			activeLayoutStore.value = layouts.active;
		};
	});
</script>

<!-- Can add categories to the layout for better organisation if needed -->

<Dialog.Root
	open={showLayoutDialog !== 'closed'}
	onOpenChange={(isOpen) => (showLayoutDialog = isOpen ? showLayoutDialog : 'closed')}
>
	<Popover.Root bind:open>
		<Popover.Trigger>
			<Button variant="outline" class="w-[200px] justify-between">
				{#if layouts.value.length > 0}
					{layouts.value[layouts.active].title}
				{:else}
					Select Layout
				{/if}
				<CaretSort class="ml-auto h-4 w-4 shrink-0 opacity-50" />
			</Button>
		</Popover.Trigger>

		<!-- Popover content -->

		<Popover.Content class="mx-2 max-h-[600px] w-[200px] p-0">
			<Command.Root>
				<Command.Input placeholder="Search layouts..." />
				<Command.List>
					<Command.Empty>No layouts found.</Command.Empty>
					<Command.Group>
						{#each layouts.value as layout, i}
							<HoverCard.Root openDelay={100} closeDelay={100}>
								<HoverCard.Trigger>
									<Command.Item onSelect={() => changeLayout(i)} value={layout.id} class="text-sm">
										{layout.title}

										{#if layouts.active === i}
											<Check class="ml-auto h-4 w-4" />
										{/if}
									</Command.Item>
								</HoverCard.Trigger>

								<!-- Not sure it they should only be displayed when there is a description -->
								<!-- {#if layout.description !== ''} -->
								<HoverCard.Content class="ml-2 p-3" side="right">
									<h2 class="font font-bold">{layout.title}</h2>
									<p>{layout.description}</p>
									<p class="opacity-30">Widget count: {layout.widgets.length}</p>
								</HoverCard.Content>
								<!-- {/if} -->
							</HoverCard.Root>
						{/each}
					</Command.Group>
				</Command.List>
				<Command.Separator />
				<Command.List>
					<Command.Group>
						<Command.Item
							onSelect={() => {
								open = false;
								showLayoutDialog = 'create';
							}}
						>
							<PlusCircled class="mr-2 h-5 w-5" />
							Create Layout
						</Command.Item>
						<Command.Item
							onSelect={() => {
								open = false;
								showLayoutDialog = 'manage';
							}}
						>
							<Pencil2 class="mr-2 h-5 w-5" />
							Manage Layouts
						</Command.Item>
					</Command.Group>
				</Command.List>
			</Command.Root>
		</Popover.Content>
	</Popover.Root>

	<Dialog.Content>
		{#if showLayoutDialog === 'create'}
			<!-- Create layout dialog content -->

			<form style="all:inherit">
				<Dialog.Header>Create Layout</Dialog.Header>
				<Label for="title">Title</Label>
				<Input id="title" class="m-auto mb-2" bind:value={createForm.title} />
				<Label for="description">Description</Label>
				<Input id="description" class="m-auto mb-2" bind:value={createForm.description} />
				<Dialog.Footer>
					<Button variant="outline" onclick={() => (showLayoutDialog = 'closed')}>Cancel</Button>
					<Button onclick={createLayout} type="submit">Create</Button>
				</Dialog.Footer>
			</form>
		{:else if showLayoutDialog === 'manage'}
			<!-- Manage layout dialog content -->

			<Dialog.Header>Manage Layouts</Dialog.Header>
			<Dialog.Description>Select a layout to edit</Dialog.Description>
			<ScrollArea class="max-h-[60vh] pl-0 pr-2">
				{#each layouts.value as layout, i}
					<Collapsible.Root class="m-2" onOpenChange={(isOpen) => handleOpenChange(i, isOpen)}>
						<Collapsible.Trigger>
							<Button
								variant="outline"
								class={cn('w-full justify-between', {
									'rounded-b-none border-b-0': openStates[i]
								})}
							>
								{layout.title}

								{#if openStates[i]}
									<ChevronUp class="ml-auto h-4 w-4 shrink-0 opacity-50" />
								{:else}
									<ChevronDown class="ml-auto h-4 w-4 shrink-0 opacity-50" />
								{/if}
							</Button>
						</Collapsible.Trigger>
						<Collapsible.Content>
							<Card.Root class={cn('mb-2 rounded-tl-none', { '-mt-[2px]': openStates[i] })}>
								<Card.Content>
									<SettingsForm {layout} />
								</Card.Content>
							</Card.Root>
						</Collapsible.Content>
					</Collapsible.Root>
				{/each}
			</ScrollArea>
		{/if}
	</Dialog.Content>
</Dialog.Root>
