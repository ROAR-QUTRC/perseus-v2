<script lang="ts" module>
	import { ros } from '$lib/scripts/ros.svelte';
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Node viewer';

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
	import Button from "$lib/components/ui/button/button.svelte";
	import { ScrollArea } from "$lib/components/ui/scroll-area/index";
	import * as Select from '$lib/components/ui/select/index';

	let nodesList = $state<string[]>([]);
	let nodeData = $state<{subscriptions: any, publications: any, services: any}>()

	const getNodes = async () => {
		ros.value!.getNodes((nodes) => {
			nodesList = nodes;
		}, (error) => {
			console.error(error);
		});
	}

	const changeSelectedNode = (value: string) => {
		ros.value!.getNodeDetails(value, (subscriptions: string[], publications: string[], services: string[]) => {
			nodeData = {subscriptions: subscriptions, publications: publications, services: services};
		}, (error) => {
			console.error(error);
		});
	}

	getNodes();
</script>

<div class="h-[100%] w-[100%] flex-col">
	<Button size="sm" variant="outline" onclick={getNodes} class="grow-0 mb-2">Refresh list</Button>
	<Select.Root type="single" onValueChange={(value) => changeSelectedNode(value)}>
		<Select.Trigger class="grow-0">Select a node</Select.Trigger>
		<Select.Content>
			{#each nodesList as node}
				<Select.Item value={node}>{node}</Select.Item>
			{/each}
		</Select.Content>
	</Select.Root>
	{#if nodeData}
		<ScrollArea orientation="both" class="p-3 grow bg-slate-500">
			<div>
				<strong>Publications:</strong>
				<div class="ml-8">
					{#each nodeData?.publications as pub}
						<p>{pub}</p>
					{/each}
				</div>
				<strong>Subscriptions:</strong>
				<div class="ml-8">
					{#each nodeData?.subscriptions as sub}
						<p>{sub}</p>
					{/each}
				</div>
				<strong>Services:</strong>
				<div class="ml-8">
					{#each nodeData?.services as serv}
						<p>{serv}</p>
					{/each}
				</div>
			</div>
		</ScrollArea>
	{/if}
</div>