<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Can Debug';
	export const description = 'Convert console CAN messages to human-readable part name';
	export const group: WidgetGroupType = 'CAN Bus';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
	
</script>

<script lang="ts">
	import { io, type Socket } from 'socket.io-client';
	import { onMount } from 'svelte';
	import { ScrollArea } from '$lib/components/ui/scroll-area/index';

	let canMsgs = $state<
		{
			address: string;
			system: string;
			subsystem: string;
			device: string;
			group: string;
			parameter: string | null;
			latestData: string[];
			timestamp: string;
		}[]
	>([]);

	// Each value object
	interface CanLutEntry {
		system: string;
		subsystem: string;
		device: string;
		group: string;
		parameter: string | null;
	}
	
	// full lookup table
	interface CanData {
		iface: string;
		address: string;
		timestamp: string;
		details: CanLutEntry;
		data: string[];
	}

	// On creation of widget
	onMount(() => {
		const socket: Socket = io();
		
		// On update from socket
		socket.on('can-data', (dataList: Array<CanData>) => {

			dataList.forEach((value, index) => {
				const inArray = canMsgs.findIndex((msg) => msg.address === value.address);

				if (inArray === -1) {
					// Add new row
					canMsgs.push({
						address: value.address,
						system: value.details.system,
						subsystem: value.details.subsystem,
						device: value.details.device,
						group: value.details.group,
						parameter: value.details.parameter,
						latestData: value.data,
						timestamp: value.timestamp,
					});
					return;
				}

				// Update data and timestamp
				canMsgs[inArray].latestData = value.data;
				canMsgs[inArray].timestamp = value.timestamp;
			})
		});

		return () => {
			socket.disconnect();
		};
	});

</script>

<ScrollArea class="h-full -m-2" orientation="vertical">
	<div class="relative h-full w-full">
		<table class="w-full table-auto">
			<thead>
				<tr class="border-b">
					<th class="border-r p-2">System</th>
					<th class="p-2">Sub System</th>
					<th class="p-2">Device</th>
					<th class="p-2">Group</th>
					<th class="p-2">Parameter</th>
					<th class="border-l p-2">Latest Data</th>
					<th class="border-l p-2">Timestamp</th>
				</tr>
			</thead>
			<tbody>
				{#each canMsgs as canMsg}
					<tr class="border border-l-0 border-x-0">
						<td class="p-2 border-r">
							<p class="w-full text-center mb-2"><b>{canMsg.system}</b></p>
						</td>
						<td class="p-2">
							<p class="w-full text-center mb-2">{canMsg.subsystem}</p>
						</td>
						<td class="p-2">
							<p class="w-full text-center mb-2">{canMsg.device}</p>
						</td>
						<td class="p-2">
							<p class="w-full text-center mb-2">{canMsg.group}</p>
						</td>
						<td class="p-2">
							<p class="w-full text-center mb-2">{canMsg.parameter}</p>
						</td>
						<td class=" w-full overflow-hidden border-l p-2">
							<p class="w-full text-center mb-2">{canMsg.latestData}</p>
						</td>
						<td class="p-2 border-l">
							<p class="w-full text-center mb-2">{canMsg.timestamp}</p>
						</td>
					</tr>
				{/each}
			</tbody>
		</table>
	</div>
</ScrollArea>
