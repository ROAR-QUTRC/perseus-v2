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
	import {Input} from '$lib/components/ui/input/index';
	import {Slider} from '$lib/components/ui/slider/index';
	import { useSliderWithInput } from '$lib/hooks/use-slider-with-input';


	const minValue = 0;
	const maxValue = 100;
	const initialValue = [25];

	const slider = useSliderWithInput({ initialValue, maxValue, minValue });

    function stopCentrifuge() {

    }
    
</script>
<div class="widget-shell">
    <h1>Space Resources Controller</h1>
    <div class="widget-content">
        <button type="button" class="danger btn" onclick={stopCentrifuge}>
            Stop Centrifuge
        </button>
        <div class="flex h-40 flex-col items-center justify-center gap-4">
		<Slider
			type="single"
			class="data-[orientation=vertical]:min-h-0"
			orientation="vertical"
			bind:value={slider.sliderValue[0]}
			onValueChange={slider.handleSliderChange}
			min={minValue}
			max={maxValue}
			aria-label="Slider with input"
		/>
		<Input
			class="h-8 w-12 px-2 py-1"
			type="text"
			inputmode="decimal"
			value={slider.inputValues[0]}
			onchange={(e) => slider.handleInputChange(e, 0)}
			onblur={() => slider.validateAndUpdateValue(slider.inputValues[0], 0)}
			onkeydown={(e) => {
				if (e.key === 'Enter') {
					slider.validateAndUpdateValue(slider.inputValues[0], 0);
				}
			}}
			aria-label="Enter value"
		/>
	</div>
    </div>
</div>
<style>     
	button {
		border: 1px solid #444;
		padding: 6px 10px;
		border-radius: 8px;
		background: #111;
		color: #eee;
		cursor: pointer;
	}
	button.danger {
		border-color: #ef4444;
	}
	button:disabled {
		opacity: 0.5;
		cursor: not-allowed;
	}
</style>
