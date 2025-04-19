<script lang="ts">
	// import { connect, isConnected, ros } from '$lib/scripts/ros.svelte';
	import { cn } from '$lib/utils';
	import * as Dialog from '$lib/components/ui/dialog/index';
	import * as DropdownMenu from '$lib/components/ui/dropdown-menu/index';
	import Button from './ui/button/button.svelte';
	import Label from './ui/label/label.svelte';
	import Input from './ui/input/input.svelte';
	import { OpenInNewWindow } from 'svelte-radix';
	import { localStore } from '$lib/scripts/localStore.svelte';
	import { connectRos, disconnectRos, getRosConnection } from '$lib/scripts/ros-bridge.svelte';

	let domainId = $state<number>(51);
	let customAddress = $state<string>('');
	let openAddressMenu = $state(false);
	let disableServerIP = $state(false);

	let address = localStore('rosAddress', 'localhost');

	const changeAddress = async (value?: string) => {
		if (value === 'server') {
			address.value = window.location.hostname; // get ip from the server
			disableServerIP = true;
		} else {
			address.value = value || customAddress; // use the custom value if there is no param
			disableServerIP = false;
		}
		disconnectRos();
		connectRos(address.value);
		openAddressMenu = false;
	};

	const toggleConnection = () => {
		if (getRosConnection()) {
			disconnectRos();
		} else {
			connectRos(address.value);
		}
	};
</script>

<div class="mx-auto flex">
	<!-- Connection -->
	<Button size="sm" variant="ghost" onclick={toggleConnection}>
		<div
			class={cn('h-[20px] w-[20px] rounded-[50%] bg-red-600', {
				'bg-green-600': getRosConnection()
			})}
		></div>
		<p>{getRosConnection() ? 'Connected' : 'Disconnected'}</p>
	</Button>

	<!-- Address -->
	<p class="my-[5px]">/</p>
	<DropdownMenu.Root>
		<DropdownMenu.Trigger class="mb-auto">
			<Button size="sm" variant="ghost">{address.value}</Button>
		</DropdownMenu.Trigger>
		<DropdownMenu.Content>
			<DropdownMenu.Item
				disabled={address.value === 'localhost'}
				onclick={() => changeAddress('localhost')}
			>
				localhost
			</DropdownMenu.Item>
			<DropdownMenu.Item onclick={() => changeAddress('server')} disabled={disableServerIP}
				>Same as host</DropdownMenu.Item
			>
			<DropdownMenu.Item onclick={() => (openAddressMenu = true)}>
				Custom IP <OpenInNewWindow class="ml-auto h-4 w-4" />
			</DropdownMenu.Item>
		</DropdownMenu.Content>
	</DropdownMenu.Root>
	<Dialog.Root bind:open={openAddressMenu}>
		<Dialog.Content>
			<Label for="customAddress">New Ros Domain ID:</Label>
			<form class="flex space-x-2">
				<Input class="m-auto mb-0" id="customAddress" type="text" bind:value={customAddress} />
				<Button type="submit" variant="outline" onclick={() => changeAddress()}>Set</Button>
			</form>
		</Dialog.Content>
	</Dialog.Root>

	<!-- ROS domain -->
	<p class="my-[5px]">/</p>
	<Button size="sm" variant="ghost" class="cursor-default">
		Domain ID: {domainId}{domainId === 42 ? ` (prod)` : ''}{domainId === 51 ? ` (dev)` : ''}
	</Button>
</div>
