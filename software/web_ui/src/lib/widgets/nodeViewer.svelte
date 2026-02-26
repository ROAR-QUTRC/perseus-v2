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
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte';
	import Button from '$lib/components/ui/button/button.svelte';
	import { ScrollArea } from '$lib/components/ui/scroll-area/index';
	import * as Select from '$lib/components/ui/select/index';
	import { Reload } from 'svelte-radix';
	import * as ROSLIB from 'roslib';
	import JsonTree from '$lib/components/jsonTree.svelte';

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

	let communicationsTree = $derived.by<{
		publications: any;
		subscriptions: any;
		services: any;
	}>(() => {
		const output = {
			publications: {} as any,
			subscriptions: {} as any,
			services: {} as any
		};

		if (!nodeData) return output;

		nodeData.publications.forEach((element) => {
			let pointer = output.publications;
			const parts = element.name.slice(1).split('/');
			for (const part of parts) {
				if (!pointer[part]) {
					if (part === parts[parts.length - 1]) {
						pointer[part] = element.type;
					} else {
						pointer[part] = {};
					}
				}
				pointer = pointer[part];
			}
		});

		nodeData.subscriptions.forEach((elements) => {
			let pointer = output.subscriptions;
			const parts = elements.name.slice(1).split('/');
			for (const part of parts) {
				if (!pointer[part]) {
					if (part === parts[parts.length - 1]) {
						pointer[part] = elements.type;
					} else {
						pointer[part] = {};
					}
				}
				pointer = pointer[part];
			}
		});

		nodeData.services.forEach((elements) => {
			let pointer = output.services;
			const parts = elements.name.slice(1).split('/');
			for (const part of parts) {
				if (!pointer[part]) {
					if (part === parts[parts.length - 1]) {
						pointer[part] = elements.type;
					} else {
						pointer[part] = {};
					}
				}
				pointer = pointer[part];
			}
		});

		return output;
	});

	const changeSelectedNode = (value: string) => {
		const ros = getRosConnection() as ROSLIB.Ros;

		selectedNode = value;
		nodeData = {
			subscriptions: [],
			publications: [],
			services: []
		};
		ros.getNodeDetails(
			value,
			(details) => {
				for (let i = 0; i < details.subscribing.length; i++) {
					ros.getTopicType(details.subscribing[i], (type) => {
						nodeData?.subscriptions.push({ name: details.subscribing[i], type });
					});
				}
				for (let i = 0; i < details.publishing.length; i++) {
					ros.getTopicType(details.publishing[i], (type) => {
						nodeData?.publications.push({ name: details.publishing[i], type });
					});
				}
				for (let i = 0; i < details.services.length; i++) {
					ros.getServiceType(details.services[i], (type) => {
						nodeData?.services.push({ name: details.services[i], type });
					});
				}
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
		<ScrollArea orientation="both" class="mt-2 h-full p-3">
			<div>
				<strong class="text-xl">Publications:</strong>
				<JsonTree data={communicationsTree.publications} />
				<strong class="text-xl">Subscriptions:</strong>
				<JsonTree data={communicationsTree.subscriptions} />
				<strong class="text-xl">Services:</strong>
				<JsonTree data={communicationsTree.services} />
			</div>
		</ScrollArea>
	</div>
</div>
