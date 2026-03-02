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

	declare module "*.png";

</script>

<script lang="ts">
	import * as ROSLIB from 'roslib';
	import { getRosConnection as getRosConnection } from '$lib/scripts/rosBridge.svelte';
    import { Slider } from '$lib/components/ui/slider/index';
	import Speedometer from "svelte-speedometer";

	type rosStringMessage = { data: string };

	//may need to change the message type
	let speedTopic: ROSLIB.Topic<rosStringMessage> | null = null;
	let controlTopic: ROSLIB.Topic<rosStringMessage> | null = null;
	let brakeTopic: ROSLIB.Topic<Boolean> | null = null;

    const minVelocity = 0;
    const maxVelocity = 100;
	let currentVelocity = $state<number>();

    let sliderValue = $state<number>(25);
    let inputValue = $state<string>('25');

    const clamp = (v: number) => Math.max(minVelocity, Math.min(maxVelocity, v));

    function handleSliderChange(next: number) {
        sliderValue = next;
        inputValue = String(next);
		//controlSpeed(number);
    }

    function handleInput(e: Event) {
        inputValue = (e.currentTarget as HTMLInputElement).value;
    }

    function commitInput() {
        const parsed = Number(inputValue);

        if (!Number.isFinite(parsed)) {
            inputValue = String(sliderValue);
            return;
        }

        const v = clamp(parsed);
        sliderValue = v;
        inputValue = String(v);

		//controlSpeed(number);
    }

    function stopCentrifuge() {
    	if (brakeTopic) {
			brakeTopic.publish(true);
		}
		else {
			new Error('ROS not connected');
		};
    }

	// function controlSpeed(speed) {
	// 	if (controlTopic) {
	// 		controlTopic.publish(speed);
	// 	}
	// 	else {
	// 		new Error('ROS not connected');
	// 	};
	// }

	$effect(() => {
		const rosConnection = getRosConnection();
		if (rosConnection) {
			controlTopic = new ROSLIB.Topic({
				ros: rosConnection,
				name: '/centrifuge/control',
				messageType:  'actuator_msgs/msg/ActuatorsAngularVelocity'
			});
			brakeTopic = new ROSLIB.Topic({
				ros: rosConnection,
				name: '/centrifuge/brake',
				messageType:  'std_msgs/msg/Bool'
			});
			speedTopic = new ROSLIB.Topic({
				ros: rosConnection,
				name: '/centrifuge/speed',
				messageType: 'actuator_msgs/msg/ActuatorsAngularVelocity'
			});
			speedTopic.subscribe(onResponseMessage);
		}
		else {
			speedTopic?.unsubscribe();
			brakeTopic = null;
			controlTopic = null;
			speedTopic = null;
		}
	});

	//update spedometer with current velocity
	const onResponseMessage = (message: rosStringMessage) => {
		const response: any = JSON.parse(message.data);
		console.log(response);
		currentVelocity = response;
	}
</script>

<div class="widget-shell">
    <div class="widget-content" style="width:100%">
        <button type="button" class="danger btn" onclick={stopCentrifuge} style="width:200px; height:100px; background-color:red; font-weight: bolder;">
            STOP CENTRIFUGE
        </button>

		<div style="margin-top:30px">
			<h1>Current Velocity</h1>
			<br>
			<Speedometer 
				maxValue={maxVelocity}
				value={sliderValue}
				needleColor="violet"
				startColor="tomato"
				endColor="lightgreen"
				segments={10}
				needleTransitionDuration={100}
			/>
		</div>

		<div>
			<h1>Velocity Control</h1>
			<div class="flex h-40 flex-col items-center justify-center gap-4">
				<Slider
					type="single"
					class="data-[orientation=vertical]:min-h-0"
					orientation="horizontal"
					value={sliderValue}
					onValueChange={handleSliderChange}
					min={minVelocity}
					max={maxVelocity}
					aria-label="Slider with input"
				/>

				<input
					class="h-8 w-12 px-2 py-1 border rounded"
					type="text"
					inputmode="decimal"
					value={inputValue}
					oninput={handleInput}
					onblur={commitInput}
					onkeydown={(e: KeyboardEvent) => {
						if (e.key === 'Enter') commitInput();
					}}
					aria-label="Enter value"
				/>
			</div>
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
