<script lang="ts" module>
	// Dynamically import all widgets
	const widgets = import.meta.glob('../lib/widgets/**.svelte', { eager: true });
	availableWidgets.length = 0;
	for (const widget in widgets) {
		availableWidgets.push({
			name: (widgets[widget] as any).name,
			settings: (widgets[widget] as any).settings,
			component: (widgets[widget] as any).default,
			description: (widgets[widget] as any).description,
			group: (widgets[widget] as any).group
		});
	}
</script>

<script lang="ts">
	import { ModeWatcher } from 'mode-watcher';
	import '../app.css';
	import { Toaster } from '$lib/components/ui/sonner';
	import { onMount } from 'svelte';
	import { connect, ros } from '$lib/scripts/ros.svelte';
	import { availableWidgets } from '$lib/scripts/state.svelte';
	import { localStore } from '$lib/scripts/localStore.svelte';
	let { children } = $props();

	// automatically connect and disconnect from ROS-bridge
	onMount(() => {
		connect(localStore('rosAddress', 'localhost').value);
		// This really should be it's own function (the return value is the onUnMount function)
		return () => {
			if (ros.value) ros.value.close();
		};
	});
</script>

<ModeWatcher defaultMode={'dark'} />
<Toaster position="bottom-left" />
{@render children()}
