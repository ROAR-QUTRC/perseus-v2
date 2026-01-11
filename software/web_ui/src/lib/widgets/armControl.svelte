<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Arm Controller';
	// These properties are optional
	export const description = 'Controller for each of the post landing arm servos';
	export const group: WidgetGroupType = 'ROS';
	// export const isRosDependent = true; // Set to true if the widget requires a ROS connection

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			general: {
				bigIncrementValue: {
					type: 'number',
					value: '30'
				},
				smallIncrementValue: {
					type: 'number',
					value: '5'
				},
				automaticallySendMessages: {
					description: 'Disables automatic sending of control messages',
					type: 'switch',
					value: 'false'
				},
				messageFrequency: {
					description: 'Frequency (Hz) to send control messages at',
					type: 'number',
					value: '10'
				}
			}
		}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import * as ROSLIB from 'roslib';
	import ScrollArea from '$lib/components/ui/scroll-area/scroll-area.svelte';
	import * as ButtonGroup from '$lib/components/ui/button-group/index';
	import Button from '$lib/components/ui/button/button.svelte';
	import Input from '$lib/components/ui/input/input.svelte';
	import Fa, { FaLayers } from 'svelte-fa';
	import { faChevronDown, faChevronUp } from '@fortawesome/free-solid-svg-icons';
	import { sentenceCase } from 'change-case';
	import { onMount, untrack } from 'svelte';
	import type { EmptyRequestType, Float64MultiArrayType, SetBoolRequestType, SetBoolResponseType } from '$lib/scripts/rosTypes';
	import type { RequestInt8ArrayResponseType, TriggerDeviceRequestType, TriggerDeviceResponseType } from '$lib/scripts/perseusMsgs';


	interface MotorData {
		showDirectPosition: boolean;
		targetAngle: number;
		draftTargetAngle: number;
		angle: number;
		rpm: number;
		temperature: number;
		current: number;
		voltage: number;
		gearRatio: Readonly<number>;
		error: string;
	}

	// ---------------------------
	//			ROS LOGIC
	// ---------------------------

	let interval: NodeJS.Timeout | null = null;
	let motorControlTopic: ROSLIB.Topic<Float64MultiArrayType> | null = null;
	let motorStatusTopic: ROSLIB.Topic<Float64MultiArrayType> | null = null;
	let motorPositionTopic: ROSLIB.Topic<Float64MultiArrayType> | null = null;
	let enableDebugStatsService: ROSLIB.Service<SetBoolRequestType, SetBoolResponseType> | null = null;
	let setIdService: ROSLIB.Service<TriggerDeviceRequestType, TriggerDeviceResponseType> | null = null;
	let setZeroPositionService: ROSLIB.Service<TriggerDeviceRequestType, TriggerDeviceResponseType> | null = null;
	let restartMotorService: ROSLIB.Service<TriggerDeviceRequestType, TriggerDeviceResponseType> | null = null;
	let getCanIdsService: ROSLIB.Service<EmptyRequestType, RequestInt8ArrayResponseType> | null = null;
	let automaticPositionMessages = $derived(settings.groups.general.automaticallySendMessages.value === 'true');

	// ROS2 connection
	$effect(() => {
		const ros = getRosConnection();
		if (ros) {
			motorControlTopic = new ROSLIB.Topic({
				ros: ros,
				name: '/arm/rmd/control',
				messageType: 'std_msgs/msg/Float64MultiArray'
			});
			motorStatusTopic = new ROSLIB.Topic({
				ros: ros,
				name: '/arm/rmd/status',
				messageType: 'std_msgs/msg/Float64MultiArray'
			});
			motorStatusTopic.subscribe(onStatusMessage)
			motorPositionTopic = new ROSLIB.Topic({
				ros: ros,
				name: '/arm/rmd/positions',
				messageType: 'std_msgs/msg/Float64MultiArray'
			});
			motorPositionTopic.subscribe(onPositionMessage)
			enableDebugStatsService = new ROSLIB.Service({
				ros: ros,
				name: '/arm/rmd/enable_debug_stats',
				serviceType: 'std_srvs/srv/SetBool'
			});
			setIdService = new ROSLIB.Service({
				ros: ros,
				name: '/arm/rmd/set_id',
				serviceType: 'perseus_msgs/srv/TriggerDevice'
			});
			setZeroPositionService = new ROSLIB.Service({
				ros: ros,
				name: '/arm/rmd/set_zero_position',
				serviceType: 'perseus_msgs/srv/TriggerDevice'
			});
			restartMotorService = new ROSLIB.Service({
				ros: ros,
				name: '/arm/rmd/restart_motor',
				serviceType: 'perseus_msgs/srv/TriggerDevice'
			});
			getCanIdsService = new ROSLIB.Service({
				ros: ros,
				name: '/arm/rmd/get_can_ids',
				serviceType: 'perseus_msgs/srv/RequestInt8Array'
			});
			getAllIds();
		} else {
			motorControlTopic = null;
		}
	});
	
	// Handle changes to settings
	$effect(() => {
		if (!automaticPositionMessages && interval) {
			clearInterval(interval);
			console.log('Cleared interval for automatic position messages');
			interval = null;
		}
		untrack(() => {
			settings.groups.general.messageFrequency.disabled = !automaticPositionMessages;
			if (automaticPositionMessages) {
				const frequency = Number(settings.groups.general.messageFrequency.value);
				interval = setInterval(() => {
					sendPositions();
				}, 1000 / frequency);
			}
		});
	});
	
	const sendPositions = () => {
		if (motorControlTopic) {
			// save draft targets to targets
			joints.forEach((joint) => {
				motors[joint].targetAngle = motors[joint].draftTargetAngle;
			});
			// send
			const message = {
				// Index of this array corresponds to motor ID - 1
				data: [
					motors.ELBOW.targetAngle * (motors.ELBOW.showDirectPosition ? 1 : motors.ELBOW.gearRatio),
					motors.WRIST_PAN.targetAngle * (motors.WRIST_PAN.showDirectPosition ? 1 : motors.WRIST_PAN.gearRatio),
					motors.WRIST_TILT.targetAngle * (motors.WRIST_TILT.showDirectPosition ? 1 : motors.WRIST_TILT.gearRatio),
					motors.SHOULDER_PAN.targetAngle * (motors.SHOULDER_PAN.showDirectPosition ? 1 : motors.SHOULDER_PAN.gearRatio),
					motors.SHOULDER_TILT.targetAngle * (motors.SHOULDER_TILT.showDirectPosition ? 1 : motors.SHOULDER_TILT.gearRatio),
				]
			};
			motorControlTopic.publish(message);
		}
	}

	const onStatusMessage = (message: Float64MultiArrayType) => {
		// message.data is an array of floats in the order:
		// [motor_id, error_code, temperature, voltage, current, rpm, angle, phase_<a,b,c>_current]
		const jointName = Object.keys(jointIdMap).find(key => jointIdMap[key as JointNameType] === message.data[0]) as JointNameType;
		if (jointName) {
			motors[jointName].error = 'None';
			motors[jointName].temperature = message.data[2];
			motors[jointName].voltage = message.data[3];
			motors[jointName].current = message.data[4];
			motors[jointName].rpm = message.data[5];
			motors[jointName].angle = message.data[6] / (motors[jointName].showDirectPosition ? 1 : motors[jointName].gearRatio);
		}
	}

	const onPositionMessage = (message: Float64MultiArrayType) => {
		// message.data is an array of floats in the order:
		// [elbow, wrist_pan, wrist_tilt, shoulder_pan, shoulder_tilt]
		motors.ELBOW.angle = message.data[0] / (motors.ELBOW.showDirectPosition ? 1 : motors.ELBOW.gearRatio);
		motors.WRIST_PAN.angle = message.data[1] / (motors.WRIST_PAN.showDirectPosition ? 1 : motors.WRIST_PAN.gearRatio);
		motors.WRIST_TILT.angle = message.data[2] / (motors.WRIST_TILT.showDirectPosition ? 1 : motors.WRIST_TILT.gearRatio);
		motors.SHOULDER_PAN.angle = message.data[3] / (motors.SHOULDER_PAN.showDirectPosition ? 1 : motors.SHOULDER_PAN.gearRatio);
		motors.SHOULDER_TILT.angle = message.data[4] / (motors.SHOULDER_TILT.showDirectPosition ? 1 : motors.SHOULDER_TILT.gearRatio);
	}

	// ---------------------------
	//			UI LOGIC
	// ---------------------------

	type JointNameType = typeof joints[number];
	
	const joints = ['SHOULDER_PAN', 'SHOULDER_TILT', 'ELBOW', 'WRIST_PAN', 'WRIST_TILT'] as const;
	const jointIdMap: Record<JointNameType, number> = {
		SHOULDER_PAN: 1,
		SHOULDER_TILT: 2,
		ELBOW: 3,
		WRIST_PAN: 4,
		WRIST_TILT: 5
	};
	let motors = $state<Record<JointNameType, MotorData>>(
		{} as Record<JointNameType, MotorData>
	);
	let onlineMotors = $state<Array<number>>([1])
	let bigIncrement = $derived(Number(settings.groups.general.bigIncrementValue.value));
	let smallIncrement = $derived(Number(settings.groups.general.smallIncrementValue.value));

	const toggleDirectOutput = (motorId: JointNameType, directButton: boolean) => {
		if (motors[motorId].showDirectPosition && directButton) return;
		if (!motors[motorId].showDirectPosition && !directButton) return;
		motors[motorId].showDirectPosition = !motors[motorId].showDirectPosition;
		const scale = motors[motorId].showDirectPosition ? motors[motorId].gearRatio : 1 / motors[motorId].gearRatio;
		motors[motorId].targetAngle = motors[motorId].targetAngle * scale;
		motors[motorId].draftTargetAngle = motors[motorId].draftTargetAngle * scale;
		motors[motorId].angle = motors[motorId].angle * scale;
	};

	const setTargetAngle = (joint: JointNameType, angle?: number) => {
		motors[joint].targetAngle = angle ?? motors[joint].draftTargetAngle;
		motors[joint].draftTargetAngle = motors[joint].targetAngle;
	}
	const incrementDraftTarget = (joint: JointNameType, increment: number) => {
		motors[joint].draftTargetAngle += increment;
	}

	const getAllIds = () => {
		if (getCanIdsService) {
			getCanIdsService.callService({}, (response) => {
				onlineMotors = response.data;
			})
		}
	}

	let getMotorIdInterval: NodeJS.Timeout | null = null;

	onMount(() => {
		// Initialize motor data
		joints.forEach((joint) => {
			motors[joint] = {
				showDirectPosition: false,
				targetAngle: 0,
				draftTargetAngle: 0,
				angle: 0,
				rpm: 0,
				temperature: 0,
				current: 0,
				voltage: 0,
				gearRatio: 10,
				error: 'None'
			};
		});

		// ensure connected motors is updated regularly
		getMotorIdInterval = setInterval(getAllIds, 1000);

		return () => {
			if (interval) {
				clearInterval(interval);
				interval = null;
			}
			if (getMotorIdInterval) {
				clearInterval(getMotorIdInterval);
				getMotorIdInterval = null;
			}
		};
	})
</script>

<div class="w-full h-full flex flex-col">
	<div>
		<Button class="mb-2" disabled={automaticPositionMessages} onclick={sendPositions}>Send Angle</Button>
		<Button class="mb-2 ml-2">Enable Debug</Button>
	</div>
	<ScrollArea class="h-full w-full pr-2 flex-1">
		<div class="h-full w-full flex flex-wrap gap-x-2">
			{#each Object.keys(motors) as motor}
				{@const data = motors[motor as JointNameType]}
				{@const id = motor as JointNameType}
				<div class="border rounded-2xl mb-2 w-fit h-fit flex flex-col p-2">
					<p class="text-xl text-center mb-2">{sentenceCase(motor)}</p>
					<div class="flex gap-2" class:disabled={!onlineMotors.includes(jointIdMap[id])}>
						<div>
							<!-- Gage -->
							<div class="border aspect-square rounded-[50%] text-center flex flex-col justify-center relative">
								<p>Angle: {data.angle}&deg;</p>
								<p>RPM: {data.rpm}</p>
								<p>Temp: {data.temperature} &deg;C</p>
								<span class="absolute border border-l-0 border-r left-1/2 top-0 h-[5%] translate-x-[-1px]"></span>
								<span class="absolute border border-l-0 border-r left-1/2 bottom-0 h-[5%] translate-x-[-1px]"></span>
								<span class="absolute border border-b-0 border-t top-1/2 w-[5%] translate-x-[-1px]"></span>
								<span class="absolute border border-b-0 border-t top-1/2 right-0 w-[5%] translate-x-[-1px]"></span>
								<svg viewBox="0 0 100 100" class="w-full h-full absolute top-0 left-0 overflow-visible">
									<circle
										r="3%"
										cx={`${50 * Math.cos((data.angle * -1 - 90) * Math.PI / 180) + 50}%`}
										cy={`${50 * Math.sin((data.angle * -1 - 90) * Math.PI / 180) + 50}%`}
										fill="red"
									/>
									<circle 
										r="3%"
										cx={`${50 * Math.cos((data.targetAngle * -1 - 90) * Math.PI / 180) + 50}%`}
										cy={`${50 * Math.sin((data.targetAngle * -1 - 90) * Math.PI / 180) + 50}%`}
										fill="red"
										opacity="0.3"
									/>
								</svg>
							</div>
							<ButtonGroup.Root class="mt-2">
								<Button class="border" 
										variant={data.showDirectPosition ? 'default' : 'outline'} 
										onclick={() => toggleDirectOutput(id, true)}>
									Direct
								</Button>
								<Button class="border" 
										variant={data.showDirectPosition ? 'outline' : 'default'} 
										onclick={() => toggleDirectOutput(id, false)}>
									Output
								</Button>
							</ButtonGroup.Root>
							<p class="opacity-40 text-center">Gear ratio: 1:{data.gearRatio}</p>
						</div>
						<div>
							<!-- Controller -->
							<ButtonGroup.Root orientation="vertical" class="max-w-[5rem]" >
								<Button variant="outline" onclick={() => incrementDraftTarget(id, bigIncrement)}>
									<FaLayers pull="left">
										<Fa translateY={0} icon={faChevronUp} />
										<Fa translateY={0.4} icon={faChevronUp} />
									</FaLayers>
								</Button>
								<Button variant="outline" onclick={() => incrementDraftTarget(id, smallIncrement)}><Fa icon={faChevronUp} /></Button>
								<Input type="number" max={32767} min={-32767} class="focus-visible:ring-1 z-10 text-center pr-0" bind:value={data.draftTargetAngle}/>
								<Button variant="outline" onclick={() => incrementDraftTarget(id, -smallIncrement)}><Fa icon={faChevronDown} /></Button>
								<Button variant="outline" onclick={() => incrementDraftTarget(id, -bigIncrement)}>
									<FaLayers pull="left">
										<Fa translateY={0} icon={faChevronDown} />
										<Fa translateY={-0.4} icon={faChevronDown} />
									</FaLayers>
								</Button>
							</ButtonGroup.Root>
						</div>
						<div class="flex flex-col">
							<!-- Extra stats -->
							<Button variant="outline" onclick={() => setTargetAngle(id, 0)}>Zero Servo</Button>
							<p>Current: {data.current.toFixed(2)} A</p>
							<p>Voltage: {data.voltage.toFixed(2)} V</p>
							<p>Error: {data.error}</p>
						</div>
					</div>
				</div>
			{/each}
		</div>
	</ScrollArea>
</div>

<style>
	.disabled {
		opacity: 0.5;
		pointer-events: none;
	}
</style>