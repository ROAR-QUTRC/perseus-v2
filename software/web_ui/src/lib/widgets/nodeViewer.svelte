<script lang="ts" module>
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Node viewer';
	export const description =
		'This widget allows you to view the topics, services, and other details of a node.';
	export const group = 'ROS';
	export const isRosDependent = true;

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/ros-bridge.svelte';
	import Button from '$lib/components/ui/button/button.svelte';
	import { ScrollArea } from '$lib/components/ui/scroll-area/index';
	import * as Select from '$lib/components/ui/select/index';
	import { Reload } from 'svelte-radix';
	import type ROSLIB from 'roslib';

	let nodesList = $state<string[]>([]);
	let nodeData = $state<{
		subscriptions: { name: string; type: string }[];
		publications: { name: string; type: string }[];
		services: { name: string; type: string }[];
	}>();
	let selectedNode = $state<string>('');

	const getNodes = async () => {
		(getRosConnection() as ROSLIB.Ros).getNodes(
			(nodes) => {
				nodesList = nodes;
			},
			(error) => {
				console.error(error);
			}
		);
	};

	const changeSelectedNode = (value: string) => {
		const ros = getRosConnection() as ROSLIB.Ros;

		selectedNode = value;
		ros.getNodeDetails(
			value,
			(subscriptions: string[], publications: string[], services: string[]) => {
				let subList: { name: string; type: string }[] = [];
				for (let i = 0; i < subscriptions.length; i++) {
					subList.push({ name: subscriptions[i], type: '' });
				}
				for (let i = 0; i < publications.length; i++) {
					ros.getTopicType(publications[i], (type) => {
						nodeData?.publications.push({ name: publications[i], type: type });
					});
				}
				for (let i = 0; i < services.length; i++) {
					ros.getServiceType(services[i], (type) => {
						nodeData?.services.push({ name: services[i], type: type });
					});
				}

				nodeData = { subscriptions: subList, publications: [], services: [] };
			},
			(error) => {
				console.error(error);
			}
		);
	};

	// When the client connects to ros bridge, get the list of nodes
	$effect(() => {
		if (getRosConnection()) {
			getNodes();
		}
	});
</script>

<div class="h-full w-full flex-col">
	<div class="flex items-center">
		<Button variant="outline" onclick={getNodes} class="mr-2 grow-0"><Reload /></Button>
		<Select.Root type="single" onValueChange={(value) => changeSelectedNode(value)}>
			<Select.Trigger class="grow-0"
				>{selectedNode === '' ? 'Select a node...' : selectedNode}</Select.Trigger
			>
			<Select.Content>
				{#each nodesList as node}
					<Select.Item value={node}>{node}</Select.Item>
				{/each}
			</Select.Content>
		</Select.Root>
	</div>

	<div class="mt-2 h-[calc(100%-40px-0.5rem)] w-full">
		{#if nodeData}
			<ScrollArea orientation="both" class="mt-2 h-full p-3">
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
								{#if sub.type !== ''}
									<kbd class="ml-1 rounded-md border px-[2px]">{sub.type}</kbd>
								{/if}
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
</div>
