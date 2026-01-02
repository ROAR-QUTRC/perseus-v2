import { type Component } from 'svelte';
import { Layout } from '../../shared/Layout';
import { WidgetSettings } from './settings.svelte';

// represents the entire widget
export interface WidgetType {
	name: string;
	description?: string;
	group?: WidgetGroupType;
	isRosDependent?: boolean;
	component: Component;
	settings: WidgetSettings;
	layoutProps?: {
		x: number;
		y: number;
		w: number;
		h: number;
	};
}

// Wrap the layouts in an object with the property: value to allow the export to be reassigned
export let layouts = $state<{ active: number; value: Array<Layout> }>({
	active: 0,
	value: []
});

export let activeWidgets = $state<{ value: Array<WidgetType> }>({ value: [] });

export let availableWidgets = $state<Array<WidgetType>>([]);

export const getWidgetByName = (name: string): WidgetType | undefined => {
	return availableWidgets.find((widget) => widget.name === name);
};

export const getWidgetsByLayoutId = (id: string): Array<WidgetType> => {
	const layout = layouts.value.find((layout) => layout.id === id);
	if (layout === undefined) return [];
	let widgets: Array<WidgetType> = [];

	console.debug('Loading widgets for layout: "', layout.title, '" Has widgets:', layout.widgets);
	for (const widgetState of layout.widgets) {
		const widgetTemplate = getWidgetByName(widgetState.name);

		console.debug('Using template for widget:', widgetState.name, widgetTemplate);

		// Widget exists if there is a template
		if (widgetTemplate) {
			// set the persisted state of each widget.
			if (widgetState.state) {
				let persistedState: WidgetSettings = JSON.parse(widgetState.state);

				console.debug('Persisted state for widget:', widgetState.name, persistedState);

				// load button actions from template
				Object.keys(widgetTemplate.settings.groups).forEach((group) => {
					Object.keys(widgetTemplate.settings.groups[group]).forEach((field) => {
						// console.log('persistedState', persistedState);
						// console.log('widgetTemplate', widgetTemplate);
						if (
							widgetTemplate.settings.groups[group][field].type === 'button' &&
							persistedState.groups[group][field].type === 'button'
						) {
							persistedState.groups[group][field].action =
								widgetTemplate.settings.groups[group][field].action;
						}
					});
				});

				widgetTemplate.settings = new WidgetSettings(persistedState.groups);
			} else {
				widgetTemplate.settings = new WidgetSettings(widgetTemplate.settings.groups);
			}

			console.debug('Final state for widget:', {
				name: widgetState.name,
				description: widgetTemplate.description,
				group: widgetTemplate.group,
				isRosDependent: widgetTemplate.isRosDependent,
				component: widgetTemplate.component,
				settings: widgetTemplate.settings,
				layoutProps: {
					x: widgetState.x,
					y: widgetState.y,
					w: widgetState.w,
					h: widgetState.h
				}
			});

			widgets.push({
				name: widgetState.name,
				description: widgetTemplate.description,
				group: widgetTemplate.group,
				isRosDependent: widgetTemplate.isRosDependent,
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

// The widget group property is a string from the list of groups defined in this type:
export type WidgetGroupType = 'ROS' | 'CAN Bus' | 'Gstreamer' | 'Misc';
