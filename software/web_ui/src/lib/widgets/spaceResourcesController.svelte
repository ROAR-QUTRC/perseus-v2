<script lang="ts" module>
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	// Internal identifiers follow: American spelling + snake_case + minimal abbreviations.
	const widget_name = 'Space Resources Controller';
	const widget_description = 'to be determined at a later date';
	const widget_group = 'ROS';
	const is_ros_dependent = true;

	const widget_settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});

	export {
		widget_name as name,
		widget_description as description,
		widget_group as group,
		is_ros_dependent as isRosDependant,
		widget_settings as settings
	};

</script>
<script lang="ts">
    // This is where the main logic of the widget will go.
</script>
<div class="widget-shell">
    <h1>Space Resources Controller</h1>
    <div class="widget-content">
        <button class="big-red-button"> Read stuff</button>

    </div>
</div>