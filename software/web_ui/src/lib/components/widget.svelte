<script lang="ts">
	import * as Card from '$lib/components/ui/card/index';
	import * as HoverCard from '$lib/components/ui/hover-card/index';
	import * as Sheet from '$lib/components/ui/sheet/index';
	import * as Select from '$lib/components/ui/select/index';
	import * as Collapsible from '$lib/components/ui/collapsible/index';
	import { CaretSort, DotsHorizontal, QuestionMarkCircled } from 'svelte-radix';
	import { layouts, type WidgetType } from '$lib/scripts/state.svelte';
	import Button from './ui/button/button.svelte';
	import Label from './ui/label/label.svelte';
	import Input from './ui/input/input.svelte';
	import Switch from './ui/switch/switch.svelte';
	import { repo } from 'remult';
	import { Layout } from '../../shared/Layout';
	import { toast } from 'svelte-sonner';
	import * as changeCase from 'change-case';

	// this is an element property
	let { widgetData, dragHandle }: { widgetData: WidgetType; dragHandle: any } = $props();

	const ComponentToRender = $derived(widgetData.component);

	let open = $state(false);

	const resetState = () => {
		toast.success('State reset');
		repo(Layout).update(layouts.value[layouts.active].id, {
			widgets: layouts.value[layouts.active].widgets.map((widget) => {
				if (widget.name === widgetData.name) {
					widget.state = '';
				}
				return widget;
			})
		});
	};

	const saveState = () => {
		toast.success('State saved');
		repo(Layout).update(layouts.value[layouts.active].id, {
			widgets: layouts.value[layouts.active].widgets.map((widget) => {
				if (widget.name === widgetData.name) {
					widget.state = JSON.stringify(widgetData.settings);
				}
				return widget;
			})
		});
	};

	const getSelectValue = (field: {
		type: 'text' | 'number' | 'select' | 'switch' | 'button';
		description?: string;
		value?: string;
		options?: { value: string; label: string }[];
		action?: () => string | null;
	}) => {
		return field.options?.find((option) => option.value === field.value)?.label;
	};

	const handleClick = (action: () => string | null) => {
		const response = action();
		if (response !== null) toast(response);
	};
</script>

<Card.Root class="h-[100%] min-h-[75px] flex-col">
	<div class="flex border-b">
		{@render dragHandle()}
		<Sheet.Root bind:open>
			<Sheet.Trigger class="outline-none">
				<DotsHorizontal class="m-2 ml-auto hover:cursor-pointer" onclick={() => (open = !open)} />
			</Sheet.Trigger>

			<!-- Settings sheet -->

			<Sheet.Content class="">
				<div class="space-y-2">
					<Sheet.Header>
						<Sheet.Title>{widgetData.name}</Sheet.Title>
					</Sheet.Header>
					{#each Object.keys(widgetData.settings.groups) as group}
						<Collapsible.Root>
							<Collapsible.Trigger>
								<Button variant="outline" class="min-w-[200px] justify-between">
									{changeCase.sentenceCase(group)}
									<CaretSort class="ml-auto h-4 w-4 shrink-0 opacity-50" />
								</Button>
							</Collapsible.Trigger>
							<Collapsible.Content class="my-2 ml-2 border-l-2 pl-2">
								<HoverCard.Root closeDelay={200} openDelay={100}>
									{#each Object.keys(widgetData.settings.groups[group]) as field}
										{#if widgetData.settings.groups[group][field].type !== 'button'}
											<div class="flex">
												<Label>{changeCase.sentenceCase(field)}</Label>
												{#if widgetData.settings.groups[group][field].description}
													<HoverCard.Trigger>
														<QuestionMarkCircled
															class="ml-2 size-4 opacity-50 hover:cursor-pointer"
														/>
													</HoverCard.Trigger>
													<HoverCard.Content class="p-2">
														{widgetData.settings.groups[group][field].description}
													</HoverCard.Content>
												{/if}
											</div>
										{/if}
										{#if widgetData.settings.groups[group][field].type === 'switch'}
											<Switch
												class="my-2"
												checked={widgetData.settings.groups[group][field].value === 'true'
													? true
													: false}
												onCheckedChange={(checked) =>
													checked
														? (widgetData.settings.groups[group][field].value = 'true')
														: (widgetData.settings.groups[group][field].value = 'false')}
											/>
											<br />
										{:else if widgetData.settings.groups[group][field].type === 'select'}
											<Select.Root
												type="single"
												name={field}
												bind:value={widgetData.settings.groups[group][field].value}
											>
												<Select.Trigger class="my-2">
													{getSelectValue(widgetData.settings.groups[group][field])}
												</Select.Trigger>
												<Select.Content>
													{#each widgetData.settings.groups[group][field].options! as option}
														<Select.Item value={option.value}>{option.label}</Select.Item>
													{/each}
												</Select.Content>
											</Select.Root>
										{:else if widgetData.settings.groups[group][field].type === 'button' && widgetData.settings.groups[group][field].action !== undefined}
											<Button
												variant="ghost"
												size="sm"
												onclick={() =>
													handleClick(widgetData.settings.groups[group][field].action!)}
											>
												{changeCase.sentenceCase(field)}
											</Button>
										{:else}
											<Input
												class="my-2"
												type={widgetData.settings.groups[group][field].type}
												bind:value={widgetData.settings.groups[group][field].value}
											/>
										{/if}
									{/each}
								</HoverCard.Root>
							</Collapsible.Content>
						</Collapsible.Root>
					{/each}
				</div>
				<div class="mt-4 flex space-x-2">
					<Button size="sm" class="w-full" onclick={saveState}>Save State</Button>
					<Button variant="outline" size="sm" class="w-full" onclick={resetState}>
						Reset State
					</Button>
				</div>
			</Sheet.Content>
		</Sheet.Root>
	</div>
	<div class="flex-grow overflow-hidden p-2" style="height: calc(100% - 40px);">
		<ComponentToRender />
	</div>
</Card.Root>
