<script lang="ts">
	import { faCaretDown, faCaretRight } from '@fortawesome/free-solid-svg-icons';
	import Fa from 'svelte-fa';

	let { data, defaultDepth = 2 } = $props<{ data: object; defaultDepth?: number }>();

	let expandedMap = $state<Record<string, boolean>>({});

	const toggleExpand = (
		event: MouseEvent,
		path: string,
		key: string,
		state: boolean,
		canCollapse: boolean
	) => {
		event.stopPropagation();
		if (!canCollapse) return; // Only toggle if it's an object
		expandedMap[`${path}/${key}`] = !state;
	};
</script>

{#snippet renderValue(value: any)}
	{#if Array.isArray(value)}
		<!-- render each array element with number -->
		<div style:color="mediumpurple" class="flex flex-col">
			{#each value as item, index}
				<p class="ml-[10px]">
					{index}: {@render renderValue(item)}{value.length - 1 !== index ? ',' : ''}
				</p>
			{/each}
		</div>
	{:else if typeof value === 'string'}
		<span style:color="deepskyblue">"{value}"</span>
	{:else if ['number', 'bigint', 'boolean'].includes(typeof value)}
		<span style="color: seagreen;">{value}</span>
	{:else}
		<span>{value}</span>
	{/if}
{/snippet}

{#snippet renderObject(data: object, depth: number = 0, path: string = '')}
	<div class="flex flex-col">
		{#each Object.entries(data) as [key, value]}
			{@const isArray = Array.isArray(value)}
			{@const containsObject = typeof value === 'object' && value !== null && !isArray}
			{@const collapsed =
				expandedMap[`${path}/${key}`] ?? (depth >= defaultDepth && (containsObject || isArray))}
			<button onclick={(e) => toggleExpand(e, path, key, collapsed, containsObject || isArray)}>
				<div
					class={`${!collapsed && (containsObject || isArray) ? 'flex-col' : 'flex-row'} ${containsObject || isArray ? '' : 'cursor-auto'} ${depth > 0 ? ' border-l  pl-[8px]' : ''} ml-[3px] mr-[16px] flex justify-start rounded-r-lg border-0 text-left hover:bg-border`}
					style:--tw-bg-opacity="0.3"
				>
					<p class="flex flex-row">
						{#if containsObject || isArray}
							<Fa class="mr-[8px] mt-[4px]" icon={collapsed ? faCaretRight : faCaretDown} />
						{/if}
						<b>{key}:&nbsp;</b>
						{#if !collapsed}
							{#if containsObject}
								<span style:color="mediumpurple">&lbrace;</span>
							{:else if isArray}
								<span style:color="mediumpurple">&lbrack;</span>
							{/if}
						{/if}
					</p>
					{#if !collapsed}
						{#if containsObject}
							<!-- recursively call to render the child object -->
							{@render renderObject(value, depth + 1, `${path}/${key}`)}
							<span style:color="mediumpurple" class="ml-[4px]">&rbrace;</span>
						{:else if isArray}
							{@render renderValue(value)}
							<span style:color="mediumpurple" class="ml-[4px]">
								&rbrack;
								<span style:opacity="0.5">({value.length})</span>
							</span>
						{:else}
							{@render renderValue(value)}
						{/if}
					{:else if containsObject}
						<p style:color="mediumpurple" style="display: inline-block;">
							&nbsp;&lbrace;&nbsp;...&nbsp;&rbrace;
						</p>
					{:else if isArray}
						<p style:color="mediumpurple" style="display: inline-block;">
							&nbsp;&lbrack;&nbsp;...&nbsp;&rbrack;
						</p>
						<span style:opacity="0.4" class="ml-[4px]">({value.length})</span>
					{/if}
				</div>
			</button>
		{/each}
	</div>
{/snippet}

{@render renderObject(data)}
