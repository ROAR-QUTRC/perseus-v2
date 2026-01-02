type SettingsFieldType =
	| {
			// Text type
			type: 'text';
			description?: string;
			value?: string;
			disabled?: boolean;
	  }
	| {
			// Number type
			type: 'number';
			description?: string;
			value?: number;
			disabled?: boolean;
	  }
	| {
			// Select type
			type: 'select';
			description?: string;
			value?: string;
			options: { value: string; label: string }[];
			disabled?: boolean;
	  }
	| {
			// Switch type
			type: 'switch';
			description?: string;
			value?: boolean;
			disabled?: boolean;
	  }
	| {
			// Button type
			type: 'button';
			description?: string;
			action: () => string | null;
			disabled?: boolean;
			value?: string;
	  }
	| {
			// Readonly type
			type: 'readonly';
			description?: string;
			value?: string;
			disabled?: false;
	  };

type SettingsFieldTypeMap = {
	text: string;
	number: number;
	select: string;
	switch: boolean;
	readonly: string;
};

// Utility type to get keys of a specific SettingsFieldType
type KeysWithType<T, Type extends SettingsFieldType['type']> = {
	[K in keyof T]: T[K] extends { type: Type } ? K : never;
}[keyof T];

// represents the settings of the widget
export type WidgetSettingsType = Record<string, Record<string, SettingsFieldType>>;

export class WidgetSettings<T extends WidgetSettingsType = WidgetSettingsType> {
	constructor(groups: T) {
		this.groups = groups;
	}

	groups = $state<WidgetSettingsType>({} as T);

	/**
	 * Get the value of a setting with correct typing
	 * @param group Group in the settings object
	 * @param field Field in the group
	 * @returns value field of the group with correct typing
	 */
	getValue<G extends keyof T = keyof T, F extends keyof T[G] = keyof T[G]>(
		group: G,
		field: F
	): T[G][F] extends { type: infer K } // return value is inferred based on the type field and the SettingsFieldTypeMap
		? K extends keyof SettingsFieldTypeMap
			? SettingsFieldTypeMap[K] | undefined
			: never
		: never {
		const setting = this.groups[group as string]?.[field as string];
		if (!setting) {
			throw new Error(`Setting not found: ${group as string}.${field as string}`);
		}
		// any is used as the function return type is explicitly defined
		return setting.value as any;
	}

	/**
	 * Set the value of a setting with correct typing
	 * @param group Group in the settings object
	 * @param field Field in the group
	 * @param value New value for the field
	 */
	setValue<G extends keyof T = keyof T, F extends keyof T[G] = keyof T[G]>(
		group: G,
		field: F,
		value: T[G][F] extends { type: infer K }
			? K extends keyof SettingsFieldTypeMap
				? SettingsFieldTypeMap[K]
				: never
			: never
	): void {
		const setting = this.groups[group as string]?.[field as string];
		if (!setting) {
			throw new Error(`Setting not found: ${group as string}.${field as string}`);
		}
		setting.value = value as any;
	}

	/**
	 * Call the action associated with a button setting programmatically
	 * @param group Group in the settings object
	 * @param field Field in the group
	 * @returns Result of the button action or undefined if the setting is not a button
	 */
	callAction<
		G extends keyof T = keyof T,
		F extends KeysWithType<T[G], 'button'> = KeysWithType<T[G], 'button'>
	>(group: G, field: F): string | null | undefined {
		const setting = this.groups[group as string]?.[field as string];
		if (!setting) {
			throw new Error(`Setting not found: ${group as string}.${field as string}`);
		}
		if (setting.type !== 'button') {
			throw new Error(`Setting is not a button: ${group as string}.${field as string}`);
		}
		return setting.action();
	}

	/**
	 * Set the action callback for a button setting
	 * @param group Group in the settings object
	 * @param field Field in the group
	 * @param callback New action callback for the button
	 */
	setActionCallback<
		G extends keyof T = keyof T,
		F extends KeysWithType<T[G], 'button'> = KeysWithType<T[G], 'button'>
	>(group: G, field: F, callback: () => string | null): void {
		const setting = this.groups[group as string]?.[field as string];
		if (!setting) {
			throw new Error(`Setting not found: ${group as string}.${field as string}`);
		}
		if (setting.type !== 'button') {
			throw new Error(`Setting is not a button: ${group as string}.${field as string}`);
		}
		setting.action = callback;
	}

	/**
	 * Disable or enable a setting field
	 * @param group Group in the settings object
	 * @param field Field in the group
	 * @param disable Should the field be disabled (Defaults to true)
	 */
	disableField<G extends keyof T = keyof T, F extends keyof T[G] = keyof T[G]>(
		group: G,
		field: F,
		disable: boolean = true
	): void {
		const setting = this.groups[group as string]?.[field as string];
		if (!setting) {
			throw new Error(`Setting not found: ${group as string}.${field as string}`);
		}
		setting.disabled = disable;
	}

	/**
	 * Add a new option to a select setting field
	 * @param group Group in the settings object
	 * @param field Field in the group
	 * @param option Value label pair to be appended to the options
	 */
	addSelectOption<
		G extends keyof T = keyof T,
		F extends KeysWithType<T[G], 'select'> = KeysWithType<T[G], 'select'>
	>(group: G, field: F, option: { value: string; label: string }): void {
		const setting = this.groups[group as string]?.[field as string];
		if (!setting) {
			throw new Error(`Setting not found: ${group as string}.${field as string}`);
		}
		if (setting.type !== 'select') {
			throw new Error(`Setting is not a select: ${group as string}.${field as string}`);
		}
		setting.options.push(option);
	}

	/**
	 * Remove an option from a select setting field
	 * @param group Group in the settings object
	 * @param field Field in the group
	 * @param optionValue Value string of the option being removed
	 */
	removeSelectOption<
		G extends keyof T = keyof T,
		F extends KeysWithType<T[G], 'select'> = KeysWithType<T[G], 'select'>
	>(group: G, field: F, optionValue: string): void {
		const setting = this.groups[group as string]?.[field as string];
		if (!setting) {
			throw new Error(`Setting not found: ${group as string}.${field as string}`);
		}
		if (setting.type !== 'select') {
			throw new Error(`Setting is not a select: ${group as string}.${field as string}`);
		}
		setting.options = setting.options.filter((opt) => opt.value !== optionValue);
	}

	/**
	 * Set the options of a select setting field
	 * @param group Group in the settings object
	 * @param field Field in the group
	 * @param options New array of value label pairs for the options
	 */
	setSelectOptions<
		G extends keyof T = keyof T,
		F extends KeysWithType<T[G], 'select'> = KeysWithType<T[G], 'select'>
	>(group: G, field: F, options: Array<{ value: string; label: string }>): void {
		const setting = this.groups[group as string]?.[field as string];
		if (!setting) {
			throw new Error(`Setting not found: ${group as string}.${field as string}`);
		}
		if (setting.type !== 'select') {
			throw new Error(`Setting is not a select: ${group as string}.${field as string}`);
		}
		setting.options = options;
	}

	/**
	 * Get the options of a select setting field
	 * @param group Group in the settings object
	 * @param field Field in the group
	 * @returns Array of value label pairs for the options
	 */
	getSelectOptions<
		G extends keyof T = keyof T,
		F extends KeysWithType<T[G], 'select'> = KeysWithType<T[G], 'select'>
	>(group: G, field: F): Array<{ value: string; label: string }> {
		const setting = this.groups[group as string]?.[field as string];
		if (!setting) {
			throw new Error(`Setting not found: ${group as string}.${field as string}`);
		}
		if (setting.type !== 'select') {
			throw new Error(`Setting is not a select: ${group as string}.${field as string}`);
		}
		return setting.options;
	}

	/**
	 * Clear all options of a select setting field
	 * @param group Group in the settings object
	 * @param field Field in the group
	 */
	clearSelectOptions<
		G extends keyof T = keyof T,
		F extends KeysWithType<T[G], 'select'> = KeysWithType<T[G], 'select'>
	>(group: G, field: F): void {
		const setting = this.groups[group as string]?.[field as string];
		if (!setting) {
			throw new Error(`Setting not found: ${group as string}.${field as string}`);
		}
		if (setting.type !== 'select') {
			throw new Error(`Setting is not a select: ${group as string}.${field as string}`);
		}
		setting.options = [];
	}
}

// Example usage:
const s = new WidgetSettings({
	General: {
		topic: {
			type: 'text',
			description: 'ROS Topic to subscribe to',
			value: '/example_topic'
		},
		enabled: {
			type: 'switch',
			description: 'Enable or disable the widget',
			value: true
		},
		refreshRate: {
			type: 'number',
			description: 'Refresh rate in Hz',
			value: 10
		},
		mode: {
			type: 'select',
			description: 'Operating mode',
			value: 'auto',
			options: [
				{ value: 'auto', label: 'Automatic' },
				{ value: 'manual', label: 'Manual' }
			]
		},
		dynamicMode: {
			type: 'select',
			description: 'Operating mode',
			options: []
		},
		resetButton: {
			type: 'button',
			description: 'Reset the widget settings',
			action: () => {
				console.log('Widget settings reset');
				return 'Reset successful';
			}
		},
		info: {
			type: 'readonly',
			description: 'Widget information',
			value: 'This is a sample widget'
		}
	}
});
