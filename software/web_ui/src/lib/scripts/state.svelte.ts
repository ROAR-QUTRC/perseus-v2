import { type Component } from 'svelte';
import { Layout } from '../../shared/Layout';

export interface WidgetType {
	name: string;
	component: Component;
	settings: WidgetSettingsType;
	layoutProps?: {
		x: number;
		y: number;
		w: number;
		h: number;
	};
}

export interface WidgetSettingsType {
	groups: Record<
		string,
		Record<
			string,
			{
				type: 'text' | 'number' | 'select' | 'switch' | 'button';
				description?: string;
				value?: string;
				options?: { value: string; label: string }[];
				action?: () => string | null;
			}
		>
	>;
}

// Wrap the layouts in an object with the property value to allow the export to be reassigned
export let layouts = $state<{ active: number; value: Array<Layout> }>({
	active: 0,
	value: []
});

export let activeWidgets = $state<{ value: Array<WidgetType> }>({ value: [] });

export let availableWidgets = $state<Array<WidgetType>>([]);

export const getWidgetByName = (
	name: string
): { component: Component; settings: WidgetSettingsType } | undefined => {
	return availableWidgets.find((widget) => widget.name === name);
};

const compareKeys = (a: WidgetSettingsType, b: WidgetSettingsType): boolean | undefined => {
	if (a && b) {
		// Deep clone the objects to prevent mutation
		let aKeys = JSON.parse(JSON.stringify(a.groups));
		let bKeys = JSON.parse(JSON.stringify(b.groups));

		Object.keys(aKeys).forEach((group) => {
			Object.keys(aKeys[group]).forEach((field) => {
				aKeys[group][field].value = undefined;
			});
		});
		Object.keys(bKeys).forEach((group) => {
			Object.keys(bKeys[group]).forEach((field) => {
				bKeys[group][field].value = undefined;
			});
		});

		return JSON.stringify(aKeys) === JSON.stringify(bKeys);
	}
	return undefined;
};

export const getWidgetsByLayoutId = (id: string): Array<WidgetType> => {
	const layout = layouts.value.find((layout) => layout.id === id);
	if (layout === undefined) return [];
	let widgets: Array<WidgetType> = [];
	for (const widget of layout.widgets) {
		const widgetData = getWidgetByName(widget.name);
		if (widgetData) {
			// set the persisted state of each widget.
			if (widget.state) {
				const persistedState: WidgetSettingsType = JSON.parse(widget.state);
				// console.log(
				// 	`Widget '${widget.name}' has a${compareKeys(widgetData.settings, persistedState) ? ' ' : 'n in'}valid persisted state`,
				// 	persistedState
				// );

				// If the state is valid and the structure of the settings has not changed, the state is applied to the settings
				if (compareKeys(widgetData.settings, persistedState)) {
					widgetData.settings.groups = persistedState.groups;
				}
				// If a change in the settings structure is detected, the state is assumed to be invalid and will be ignored
				else {
					widget.state = undefined;
				}
			}

			widgets.push({
				name: widget.name,
				component: widgetData.component,
				settings: widgetData.settings,
				layoutProps: {
					x: widget.x,
					y: widget.y,
					w: widget.w,
					h: widget.h
				}
			});
		}
	}

	return widgets;
};
