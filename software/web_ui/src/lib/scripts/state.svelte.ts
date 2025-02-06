import { type Component } from 'svelte';
import { Layout } from '../../shared/Layout';

// represents the entire widget
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

// represents the settings of the widget
export interface WidgetSettingsType {
	groups: Record<
		string,
		Record<
			string,
			{
				type: 'text' | 'number' | 'select' | 'switch' | 'button' | 'readonly';
				description?: string;
				value?: string;
				options?: { value: string; label: string }[]; // these are not saved as options are typically session dependent
				disabled?: boolean;
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

export const getWidgetsByLayoutId = (id: string): Array<WidgetType> => {
	const layout = layouts.value.find((layout) => layout.id === id);
	if (layout === undefined) return [];
	let widgets: Array<WidgetType> = [];

	for (const widgetState of layout.widgets) {
		const widgetTemplate = getWidgetByName(widgetState.name);

		// Widget exists if there is a template
		if (widgetTemplate) {
			// set the persisted state of each widget.
			if (widgetState.state) {
				let persistedState: WidgetSettingsType = JSON.parse(widgetState.state);

				// load button actions from template
				Object.keys(widgetTemplate.settings.groups).forEach((group) => {
					Object.keys(widgetTemplate.settings.groups[group]).forEach((field) => {
						if (
							widgetTemplate.settings.groups[group][field].type === 'button' &&
							persistedState.groups[group]
						) {
							persistedState.groups[group][field].action =
								widgetTemplate.settings.groups[group][field].action;
						}
					});
				});

				widgetTemplate.settings.groups = persistedState.groups;
			}

			widgets.push({
				name: widgetState.name,
				component: widgetTemplate.component,
				settings: widgetTemplate.settings,
				layoutProps: {
					x: widgetState.x,
					y: widgetState.y,
					w: widgetState.w,
					h: widgetState.h
				}
			});
		}
	}

	return widgets;
};
