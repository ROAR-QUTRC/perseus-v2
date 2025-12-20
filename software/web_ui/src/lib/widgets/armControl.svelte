<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Arm Controller';
	// These properties are optional
	export const description = 'Controller for each of the post landing arm servos';
	export const group: WidgetGroupType = 'ROS';
	export const isRosDependent = true; // Set to true if the widget requires a ROS connection

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			general: {
				disableSendForm: {
					type: 'switch',
					value: 'false',
				},
				topic: {
					type: 'text',
					value: '',
				},
				incrementValue: {
					type: 'number',
					value: '5',
				}
			}
		}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/ros-bridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import ROSLIB from 'roslib';
	import { onMount } from 'svelte';
	import { Button } from '$lib/components/ui/button/index.js';
	import { Slider } from '$lib/components/ui/slider/index.js';

	let topic: ROSLIB.Topic | null = null;
	let elbowPosition = $state(0);
	let wristYawPosition = $state(0);
	let wristRollPosition = $state(0);
	let shoulderPitchPosition = $state(0);
	let shoulderYawPosition = $state(0);

	$effect(() => {
		const ros = getRosConnection();
		if(ros){
			const topicName = settings.groups.general.topic.value || '/arm/rmd_control';
			ros.getTopicType(topicName, (type) => {
				topic = new ROSLIB.Topic({
					ros: ros,
					name: topicName,
					messageType: type,
				});
				if(type !== "perseus_msgs/ArmServoControl"){
					settings.groups.general.disableSendForm.value === 'false';
					settings.groups.general.disableSendForm.disabled = true;
				} else{
					settings.groups.general.disableSendForm.disabled = false;
				}
				topic.advertise();
			})
		}else{
			topic = null;
		}
	})
	const sendMessage = (e: Event) => {
		e.preventDefault();
		console.log("Hello");
	};

</script>

<div class="w-full h-full flex flex-col">
	<div style="text-align:center">
		Shoulder Pitch Position: {shoulderPitchPosition}
	</div>
	<div class="flex flex-row">
		<Button onclick={() => {shoulderPitchPosition = shoulderPitchPosition - Number(settings.groups.general.incrementValue.value); if(shoulderPitchPosition < 0){shoulderPitchPosition = 0;}}}>Decrement</Button>
		<Slider type="single" bind:value={shoulderPitchPosition} max={100} step={1}/>
		<Button onclick={() => {shoulderPitchPosition = shoulderPitchPosition + Number(settings.groups.general.incrementValue.value); if(shoulderPitchPosition > 100){shoulderPitchPosition = 100;}}}>Increment</Button>
	</div>
	<br>
	<div style="text-align:center">
		Shoulder Yaw Position: {shoulderYawPosition}
	</div>
	<div class="flex flex-row">
		<Button onclick={() => {shoulderYawPosition = shoulderYawPosition - Number(settings.groups.general.incrementValue.value); if(shoulderYawPosition < 0){shoulderYawPosition = 0;}}}>Decrement</Button>
		<Slider type="single" bind:value={shoulderYawPosition} max={100} step={1}/>
		<Button onclick={() => {shoulderYawPosition = shoulderYawPosition + Number(settings.groups.general.incrementValue.value); if(shoulderYawPosition > 100){shoulderYawPosition = 100;}}}>Increment</Button>
	</div>
	<br>
	<div style="text-align:center">
		Elbow Position: {elbowPosition}
	</div>
	<div class="flex flex-row">
		<Button onclick={() => {elbowPosition = elbowPosition - Number(settings.groups.general.incrementValue.value); if(elbowPosition < 0){elbowPosition = 0;}}}>Decrement</Button>
		<Slider type="single" bind:value={elbowPosition} max={100} step={1}/>
		<Button onclick={() => {elbowPosition = elbowPosition + Number(settings.groups.general.incrementValue.value); if(elbowPosition > 100){elbowPosition = 100;}}}>Increment</Button>
	</div>
	<br>
	<div style="text-align:center">
		Wrist Yaw Position: {wristYawPosition}
	</div>
	<div class="flex flex-row">
		<Button onclick={() => {wristYawPosition = wristYawPosition - Number(settings.groups.general.incrementValue.value); if(wristYawPosition < 0){wristYawPosition = 0;}}}>Decrement</Button>
		<Slider type="single" bind:value={wristYawPosition} max={100} step={1}/>
		<Button onclick={() => {wristYawPosition = wristYawPosition + Number(settings.groups.general.incrementValue.value); if(wristYawPosition > 100){wristYawPosition = 100;}}}>Increment</Button>
	</div>
	<br>
	<div style="text-align:center">
		Wrist Roll Position: {wristRollPosition}
	</div>
	<div class="flex flex-row">
		<Button onclick={() => {wristRollPosition = wristRollPosition - Number(settings.groups.general.incrementValue.value); if(wristRollPosition < 0){wristRollPosition = 0;}}}>Decrement</Button>
		<Slider type="single" bind:value={wristRollPosition} max={100} step={1}/>
		<Button onclick={() => {wristRollPosition = wristRollPosition + Number(settings.groups.general.incrementValue.value); if(wristRollPosition > 100){wristRollPosition = 100;}}}>Increment</Button>
	</div>
</div>