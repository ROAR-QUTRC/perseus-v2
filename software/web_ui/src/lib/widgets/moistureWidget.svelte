<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Kibisis Moisture Sensor';
	// These properties are optional
	export const description = 'Description of the widget goes here';
	export const group: WidgetGroupType = 'Kibisis';
	export const isRosDependent = true; // Set to true if the widget requires a ROS connection

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import * as ROSLIB from 'roslib';
	import type {
		EmptyRequestType,
		NumberMessageType,
		SetBoolResponseType
	} from '$lib/scripts/rosTypes';
	import Button from '$lib/components/ui/button/button.svelte';

	type TopicIdTypes =
		| '/kibisis/ldr/a_ambient'
		| '/kibisis/ldr/b_ambient'
		| '/kibisis/ldr/a_illuminated'
		| '/kibisis/ldr/b_illuminated';

	let requestSuccess = $state<true | string | null>(null);

	let data: Record<TopicIdTypes, number | null> = {
		'/kibisis/ldr/a_ambient': null,
		'/kibisis/ldr/b_ambient': null,
		'/kibisis/ldr/a_illuminated': null,
		'/kibisis/ldr/b_illuminated': null
	};

	// topic name mapped to instance of ROSLIB.Topic
	let topicHandles: Record<TopicIdTypes, ROSLIB.Topic<NumberMessageType> | null> = {
		'/kibisis/ldr/a_ambient': null,
		'/kibisis/ldr/b_ambient': null,
		'/kibisis/ldr/a_illuminated': null,
		'/kibisis/ldr/b_illuminated': null
	};
	let triggerService: ROSLIB.Service<EmptyRequestType, SetBoolResponseType> | null = null;

	$effect(() => {
		const ros = getRosConnection();

		if (ros) {
			// Initialize topic handles
			Object.entries(topicHandles).forEach(([topicName, _]) => {
				topicHandles[topicName as TopicIdTypes] = new ROSLIB.Topic({
					ros,
					name: topicName,
					messageType: 'std_msgs/Float32'
				});

				topicHandles[topicName as TopicIdTypes]?.subscribe((message) => {
					data[topicName as TopicIdTypes] = message.data;
				});
			});

			// Initialize service handle
			triggerService = new ROSLIB.Service({
				ros,
				name: '/kibisis/ldr/sample',
				serviceType: 'std_srvs/srv/Trigger'
			});
		}
	});

	const triggerSample = () => {
		if (triggerService) {
			triggerService.callService({}, (response) => {
				if (response.success) requestSuccess = true;
				else requestSuccess = response.message;
			});
		}
	};
</script>

<div>
	<Button onclick={triggerSample}>Yoink some data</Button>
	<p>{requestSuccess === true ? 'Success!' : requestSuccess}</p>
</div>

<div>
	{#each Object.entries(data) as [topicName, value]}
		<div>
			<strong>{topicName}</strong>
			<p>{value !== null ? value : 'No data yet'}</p>
		</div>
	{/each}
</div>
