<script lang="ts">
	import { connect, isConnected, ros } from '$lib/scripts/ros.svelte';
	import { cn } from '$lib/utils';
	import * as Dialog from '$lib/components/ui/dialog/index';
	import * as DropdownMenu from '$lib/components/ui/dropdown-menu/index';
	import Button from './ui/button/button.svelte';
	import Label from './ui/label/label.svelte';
	import Input from './ui/input/input.svelte';
	import { OpenInNewWindow } from 'svelte-radix';
	import { localStore } from '$lib/scripts/localStore.svelte';

	let domainId = $state<number>(51);
	let customDomain = $state<number>(0);
	let openDomainMenu = $state(false);
	let customAddress = $state<string>('');
	let openAddressMenu = $state(false);
	let disableServerIP = $state(false);

	let address = localStore('rosAddress', 'localhost');

	const changeDomainId = (id: number) => () => {
		fetch('/api/ros/domain', {
			method: 'POST',
			headers: {
				'Content-Type': 'application/json'
			},
			body: JSON.stringify({ domainId: id })
		}).then(() => {
			domainId = id;
		});
	};

	const changeAddress = async (value?: string) => {
		if (value === 'server') {
			const res = await fetch('/api/ros/domain', { method: 'GET' });
			const data = await res.json();
			address.value = data; // get ip from the server
			disableServerIP = true;
		} else {
			address.value = value || customAddress; // use the custom value if there is no param
			disableServerIP = false;
		}

		ros.value?.close();
		connect(address.value);
		openAddressMenu = false;
	};
</script>

<div class="mx-auto flex">
	<!-- Connection -->
	<DropdownMenu.Root>
		<DropdownMenu.Trigger class="mb-auto">
			<Button size="sm" variant="ghost">
				<div
					class={cn('my-[8px] h-[20px] w-[20px] rounded-[50%] bg-red-600', {
						'bg-green-600': isConnected()
					})}
				></div>
				<p>{isConnected() ? 'Connected' : 'Disconnected'}</p>
			</Button>
		</DropdownMenu.Trigger>
		<DropdownMenu.Content>
			<DropdownMenu.Item onclick={() => ros.value?.close()} disabled={!isConnected()}>
				Disconnect
			</DropdownMenu.Item>
			<DropdownMenu.Item onclick={() => connect(address.value)} disabled={isConnected()}>
				Connect
			</DropdownMenu.Item>
		</DropdownMenu.Content>
	</DropdownMenu.Root>

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
	<DropdownMenu.Root>
		<DropdownMenu.Trigger class="mb-auto">
			<Button size="sm" variant="ghost">
				Domain ID: {domainId}{domainId === 42 ? ` (prod)` : ''}{domainId === 51 ? ` (dev)` : ''}
			</Button>
		</DropdownMenu.Trigger>
		<DropdownMenu.Content>
			<DropdownMenu.Item onclick={changeDomainId(51)}>Development</DropdownMenu.Item>
			<DropdownMenu.Item onclick={changeDomainId(42)}>Production</DropdownMenu.Item>
			<DropdownMenu.Item onclick={() => (openDomainMenu = true)}>
				Custom <OpenInNewWindow class="ml-auto h-4 w-4" />
			</DropdownMenu.Item>
		</DropdownMenu.Content>
	</DropdownMenu.Root>
	<Dialog.Root bind:open={openDomainMenu}>
		<Dialog.Content>
			<Label for="customDomain">New Ros Domain ID:</Label>
			<form class="flex space-x-2">
				<Input class="m-auto mb-0" id="customDomain" type="number" bind:value={customDomain} />
				<Button type="submit" variant="outline" onclick={changeDomainId(customDomain)}>Set</Button>
			</form>
		</Dialog.Content>
	</Dialog.Root>
</div>
