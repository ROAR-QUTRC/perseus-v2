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
	import type { ActuatorsMessageType, AngularVelocityMessageType, EmptyRequestType, Float64MultiArrayType} from '$lib/scripts/rosTypes';

	type rosBooleanMessage = { data: boolean };
	type rosFloatMessage = { data: number };

	//declare ROS topics
	let speedTopic: ROSLIB.Topic<AngularVelocityMessageType> | null = null;
	let waterConcentrationTopic: ROSLIB.Topic<rosFloatMessage> | null = null;
	let ilmeniteConcentrationTopic: ROSLIB.Topic<rosFloatMessage> | null = null;
	let controlTopic: ROSLIB.Topic<AngularVelocityMessageType> | null = null;
	let brakeTopic: ROSLIB.Topic<rosBooleanMessage> | null = null;
	let getWaterConcentration: ROSLIB.Topic<EmptyRequestType, Float64MultiArrayType> | null = null;
	let getIlmeniteConcentration: ROSLIB.Topic<EmptyRequestType, Float64MultiArrayType> | null = null;

    const minVelocity = 0;
    const maxVelocity = 100;
	let currentVelocity = $state<number>();
	let waterConcentration = $state<number>();
	let ilmeniteConcentration = $state<number>();

    let sliderValue = $state<number>(25);
    let inputValue = $state<string>('25');

    const clamp = (v: number) => Math.max(minVelocity, Math.min(maxVelocity, v));

	//adjusts desired centrifuge velocity based on slider position
    function handleSliderChange(next: number) {
        sliderValue = next;
        inputValue = String(next);
		controlSpeed(next);
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
			brakeTopic.publish({data: true});
		}
		else {
			new Error('ROS not connected');
		};
    }

	function controlSpeed(speed: number) {
		if (controlTopic) {
			const message: AngularVelocityMessageType = {
				velocity: [speed]
			};

			controlTopic.publish(message);
		}
		else {
			new Error('ROS not connected');
		};
	}

	//request water concentration reading from controller node
	const takeWaterReading = () => {
		console.log("water");
		if (getWaterConcentration) {
			getWaterConcentration.callService({}, (response: Float64Array) => {})
		}
	}

	//request ilmenite concentration reading from controller node
	const takeIlmeniteReading = () => {
		console.log("ilmenite");
		if (getIlmeniteConcentration) {
			getIlmeniteConcentration.callService({}, (response: Float64Array) => {})
		}
	}

	//establish ROS publishers, subscribers and services
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
			speedTopic.subscribe(onSpeedMessage);
			getWaterConcentration = new ROSLIB.Service({
				ros: rosConnection,
				name: '/water/reading',
				serviceType: 'perseus_interfaces/srv/Concentration'
			});
			getIlmeniteConcentration = new ROSLIB.Service({
				ros: rosConnection,
				name: '/ilmenite/reading',
				serviceType: 'perseus_interfaces/srv/Concentration'
			});
			waterConcentrationTopic = new ROSLIB.Topic({
				ros: rosConnection,
				name: '/water_concentration/result',
				messageType: 'std_msgs/msg/Float64'
			});
			waterConcentrationTopic.subscribe(onWaterMessage);
			ilmeniteConcentrationTopic = new ROSLIB.Topic({
				ros: rosConnection,
				name: '/ilmenite_concentration/result',
				messageType: 'std_msgs/msg/Float64'
			});
			ilmeniteConcentrationTopic.subscribe(onIlmeniteMessage);
		}
		else {
			speedTopic?.unsubscribe();
			waterConcentrationTopic?.unsubscribe();
			ilmeniteConcentrationTopic?.unsubscribe();
			brakeTopic = null;
			controlTopic = null;
			speedTopic = null;
		}
	});

	//update spedometer with current velocity
	const onSpeedMessage = (message: ActuatorsMessageType) => {
		let response = message.velocity;
		console.log(response);
		currentVelocity = response[0];
	}

	//update spedometer with current water concentration
	const onWaterMessage = (message: rosFloatMessage) => {
		let response = message;
		console.log(response.data);
		waterConcentration = response.data;
	}

	//update spedometer with current ilmenite concentration
	const onIlmeniteMessage = (message: rosFloatMessage) => {
		let response = message;
		console.log(response);
		ilmeniteConcentration = response.data;
	}

</script>

<div class="widget-shell">
    <div class="widget-content" style="width:100%">
		<div style="display:flex; flex-direction: row; width:100%; justify-content: space-around; align-items:center">
			<button type="button" class="danger btn" onclick={stopCentrifuge} style="width:250px; height:150px; background-color:red; font-weight: bolder;">
				STOP CENTRIFUGE
			</button>

			<div style="margin-top:30px; height:250px">
				<h1>Current Velocity</h1>
				<br>
				<Speedometer 
					maxValue={maxVelocity}
					value={currentVelocity}
					needleColor="violet"
					startColor="tomato"
					endColor="lightgreen"
					segments={10}
					needleTransitionDuration={100}
				/>
			</div>
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
		<div style="display:flex; flex-direction: row; width:100%; justify-content: space-around; align-items:center">
			<div style="margin-top:30px">
				<h1>Water Concentration (%)</h1>
				<button type="button" class="btn" onclick={takeWaterReading} style="margin-top:10px; margin-bottom:20px; background-color:violet;">
					Take Water Reading
				</button>
				<Speedometer 
					maxValue={100}
					value={waterConcentration}
					needleColor="violet"
					startColor="lightblue"
					endColor="darkblue"
					segments={10}
					needleTransitionDuration={100}
				/>
			</div>

			<div style="margin-top:30px">
				<h1>Ilmenite Concentration (%)</h1>
				<button type="button" class="btn" onclick={takeIlmeniteReading} style="margin-top:10px; margin-bottom:20px; background-color:violet;">
					Take Ilmenite Reading
				</button>
				<Speedometer 
					maxValue={100}
					value={ilmeniteConcentration}
					needleColor="violet"
					startColor="white"
					endColor=#22262b
					segments={10}
					needleTransitionDuration={100}
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
