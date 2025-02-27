<script lang="ts" module>
	import { ros } from '$lib/scripts/ros.svelte';
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Node viewer';
	export const description =
		'This widget allows you to view the topics, services, and other details of a node.';
	export const group = 'ROS';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	import Button from '$lib/components/ui/button/button.svelte';
	import { ScrollArea } from '$lib/components/ui/scroll-area/index';
	import * as Select from '$lib/components/ui/select/index';
	import { Reload } from 'svelte-radix';

	let nodesList = $state<string[]>([]);
	let nodeData = $state<{
		subscriptions: { name: string; type: string }[];
		publications: { name: string; type: string }[];
		services: { name: string; type: string }[];
	}>();

	const getNodes = async () => {
		ros.value!.getNodes(
			(nodes) => {
				nodesList = nodes;
			},
			(error) => {
				console.error(error);
			}
		);
	};

	const changeSelectedNode = (value: string) => {
		// ros.value!.getTopicsAndRawTypes(
		// 	(topics) => {
		// 		console.log(topics);
		// 	},
		// 	(error) => {
		// 		console.error(error);
		// 	}
		// );

		ros.value!.getNodeDetails(
			value,
			(subscriptions: string[], publications: string[], services: string[]) => {
				let subList: { name: string; type: string }[] = [];
				for (let i = 0; i < subscriptions.length; i++) {
					ros.value!.getTopicType(subscriptions[i], (type) => {
						subList.push({ name: subscriptions[i], type: type });
					});
				}
				for (let i = 0; i < publications.length; i++) {
					ros.value!.getTopicType(publications[i], (type) => {
						nodeData?.publications.push({ name: publications[i], type: type });
					});
				}
				for (let i = 0; i < services.length; i++) {
					ros.value!.getServiceType(services[i], (type) => {
						nodeData?.services.push({ name: services[i], type: type });
					});
				}

				nodeData = { subscriptions: subList, publications: [], services: [] };

				// console.log(nodeData);
				// nodeData = { subscriptions: subscriptions, publications: publications, services: services };
			},
			(error) => {
				console.error(error);
			}
		);
	};

	getNodes();
</script>

<div class="h-[100%] w-[100%] flex-col">
	<div class="flex items-center">
		<Button variant="outline" onclick={getNodes} class="mr-2 grow-0"><Reload /></Button>
		<Select.Root type="single" onValueChange={(value) => changeSelectedNode(value)}>
			<Select.Trigger class="grow-0">Select a node</Select.Trigger>
			<Select.Content>
				{#each nodesList as node}
					<Select.Item value={node}>{node}</Select.Item>
				{/each}
			</Select.Content>
		</Select.Root>
	</div>

	{#if nodeData}
		<ScrollArea orientation="both" class="mt-2 grow rounded-md border p-3">
			<div>
				<strong>Publications:</strong>
				<div class="ml-8">
					{#each nodeData?.publications as pub}
						<div class="mb-1 flex">
							<p>{pub.name}</p>
							<kbd class="ml-1 rounded-md border px-[2px]">{pub.type}</kbd>
						</div>
					{/each}
				</div>
				<strong>Subscriptions:</strong>
				<div class="ml-8">
					{#each nodeData?.subscriptions as sub}
						<div class="mb-1 flex">
							<p>{sub.name}</p>
							<kbd class="ml-1 rounded-md border px-[2px]">{sub.type}</kbd>
						</div>
					{/each}
				</div>
				<strong>Services:</strong>
				<div class="ml-8">
					{#each nodeData?.services as serv}
						<div class="mb-1 flex">
							<p>{serv.name}</p>
							<kbd class="ml-1 rounded-md border px-[2px]">{serv.type}</kbd>
						</div>
					{/each}
				</div>
			</div>
		</ScrollArea>
	{/if}
</div>
