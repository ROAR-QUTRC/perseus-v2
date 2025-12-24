<script lang="ts">
	import { Button } from '$lib/components/ui/button/index';
	import { Check, Copy, LockClosed, LockOpen2, Trash } from 'svelte-radix';
	import Label from '$lib/components/ui/label/label.svelte';
	import Input from '$lib/components/ui/input/input.svelte';
	import { repo } from 'remult';
	import { Layout } from '../../shared/Layout';
	import { layouts } from '$lib/scripts/state.svelte';

	let { layout } = $props();

	const toggleLock = () => {
		repo(Layout).update(layout.id, { isLocked: !layout.isLocked });
	};

	let settingsForm = $state<{ title: string; description: string }>({
		title: layout.title,
		description: layout.description
	});

	const updateLayoutTitle = () => {
		repo(Layout).update(layout.id, { title: settingsForm.title });
	};

	const updateLayoutDescription = () => {
		repo(Layout).update(layout.id, { description: settingsForm.description });
	};

	let readyToDelete = $state(false);
	const deleteLayout = () => {
		repo(Layout).delete(layout.id);
		layouts.active = 0;
		readyToDelete = false;
	};
</script>

<Label for="title">Title</Label>
<form class="flex space-x-2">
	<Input id="title" class="m-auto mb-2" bind:value={settingsForm.title} />
	<Button variant="outline" size="icon" type="submit" onclick={updateLayoutTitle}>
		<Check />
	</Button>
</form>
<Label for="description">Description</Label>
<form class="flex space-x-2">
	<Input id="description" class="m-auto mb-2" bind:value={settingsForm.description} />
	<Button variant="outline" size="icon" type="submit" onclick={updateLayoutDescription}>
		<Check />
	</Button>
</form>
<Label>Actions</Label>
<div class="flex space-x-2">
	<Button variant="outline" size="icon" disabled>
		<Copy />
	</Button>
	<Button variant="outline" size="icon" onclick={toggleLock}>
		{#if layout.isLocked}
			<LockClosed />
		{:else}
			<LockOpen2 />
		{/if}
	</Button>
	{#if !readyToDelete}
		<Button variant="outline" size="icon" onclick={() => (readyToDelete = true)}>
			<Trash />
		</Button>
	{:else}
		<Button variant="outline" onclick={() => (readyToDelete = false)}>Cancel</Button>
		<Button variant="destructive" onclick={deleteLayout}>Delete: {layout.title}</Button>
	{/if}
</div>
<Label class="text-red-500">The copy action dont work :'(</Label>
