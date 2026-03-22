<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Arm End Effector';
	// These properties are optional
	export const description =
		'Control and view the analog signals sent and received from the end effector';
	export const group: WidgetGroupType = 'ROS';
	export const isRosDependent = true; // Set to true if the widget requires a ROS connection

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			channelA: {
				label: {
					type: 'text',
					value: 'Channel A'
				},
				output: {
					type: 'switch',
					value: 'false'
				}
			},
			channelB: {
				label: {
					type: 'text',
					value: 'Channel B'
				},
				output: {
					type: 'switch',
					value: 'false'
				}
			},
			channelC: {
				label: {
					type: 'text',
					value: 'Channel C'
				},
				output: {
					type: 'switch',
					value: 'false'
				}
			},
			channelD: {
				label: {
					type: 'text',
					value: 'Channel D'
				},
				output: {
					type: 'switch',
					value: 'false'
				}
			}
		}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import * as ROSLIB from 'roslib';
	import Slider from '$lib/components/ui/slider/slider.svelte';
	import type { NumberArrayType } from '$lib/scripts/rosTypes';
	import { untrack } from 'svelte';

	// Generate map based off settings object so number of channels is entirely dependent on the settings
	const CHANNEL_KEY_INDEX_MAP: Record<string, number> = Object.entries(settings.groups).reduce(
		(acc, [key, _], index) => {
			acc[key] = index;
			return acc;
		},
		{} as Record<string, number>
	);

	let data = $state<Array<{ label: string; output: boolean; value: number }>>([]);

	// use effect to update label and direction while still being able to bind to $state rune
	$effect(() => {
		const output: Array<{ label: string; output: boolean; value: number }> = [];
		Object.entries(settings.groups).forEach(([key, group]) => {
			// next value is dependent on previous value, so untrack to prevent infinite loop
			untrack(() => {
				output.push({
					label: group.label.value || String(CHANNEL_KEY_INDEX_MAP[key]),
					output: group.output.value === 'true',
					value: data[CHANNEL_KEY_INDEX_MAP[key]]?.value ?? 0 // preserve slider value if it exists, otherwise default to 0
				});
			});
		});
		data = output;
	});

	// handling ROS2 topic subscription and publication
	let read: ROSLIB.Topic<NumberArrayType> | null = null;
	let write: ROSLIB.Topic<NumberArrayType> | null = null;

	$effect(() => {
		const ros = getRosConnection();
		if (!ros) return;

		read = new ROSLIB.Topic({
			ros,
			name: '/arm/end_effector/read',
			messageType: 'std_msgs/msg/UInt16MultiArray'
		});

		write = new ROSLIB.Topic({
			ros,
			name: '/arm/end_effector/write',
			messageType: 'std_msgs/msg/UInt16MultiArray'
		});

		read.subscribe((message) => {
			message.data.forEach((value, index) => {
				if (index < data.length) {
					if (data[index].output) data[index].value = (value / 65535) * 100; // map 16bit int to percentage for slider
				}
			});
		});
	});

	const writeValue = (channelIndex: number, percentage: number) => {
		if (!write) return;
		// topic takes all values at once
		const pwmValues = data.map((channel) => {
			if (!channel.output) return Math.round((channel.value / 100) * 65535); // map percentage to 16bit int for ROS topic
			return 0; // if channel is not an output, send 0
		});
		pwmValues[channelIndex] = Math.round((percentage / 100) * 65535); // update the changed value
		write.publish({ data: pwmValues });
	};
</script>

<h3 class="text-lg"><strong>Inputs:</strong></h3>
{#if data.filter((channel) => !channel.output).length > 0}
	<!-- Use if instead of iterating filtered list as filter returns a new array -->
	{#each data as channel, index}
		{#if !channel.output}
			<!-- Render sliders for end effector inputs -->
			<div class={channel.output ? 'pointer-events-none opacity-30' : ''}>
				<p>{channel.label}{index === 0 ? ' (Servo)' : ''}: {channel.value.toFixed()}%</p>
				<!-- onValueCommit is used to prevent flooding CAN with intermediate values -->
				<Slider
					onValueCommit={(e) => writeValue(index, e)}
					class="my-2"
					disabled={channel.output}
					type="single"
					max={100}
					min={0}
					bind:value={data[index].value}
				/>
			</div>
		{/if}
	{/each}
{:else}
	<p>Use the widget settings to configure inputs.</p>
{/if}

<h3 class="mt-4 text-lg"><strong>Outputs:</strong></h3>
{#each data.filter((channel) => channel.output) as channel, index}
	<!-- Render value readouts for end effector outputs -->
	<div class="flex flex-row items-center gap-3">
		<p>{channel.label}: {channel.value.toFixed()}%</p>
		<div class="rounded-1/2 h-[6px] flex-1 rounded-[3px] border">
			<span
				class="block h-full rounded-[3px] bg-primary transition-all duration-300"
				style:width={`${channel.value.toFixed()}%`}
			></span>
		</div>
	</div>
{:else}
	<p>Use the widget settings to configure outputs.</p>
{/each}
