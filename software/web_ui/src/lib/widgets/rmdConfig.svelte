<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetGroupType } from '$lib/scripts/state.svelte';
	import { WidgetSettings } from '$lib/scripts/settings.svelte';


	export const name = 'RMD Config Widget';
	// These properties are optional
	export const description = 'Configure and monitor RMD motors connected via CAN';
	export const group: WidgetGroupType = 'CAN Bus';
	export const isRosDependent = true; // Set to true if the widget requires a ROS connection

	export const settings = $state(new WidgetSettings({}));
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import * as ROSLIB from 'roslib';
	import { onMount } from 'svelte';
	import * as Select from '$lib/components/ui/select';
	import Button from '$lib/components/ui/button/button.svelte';
	import Fa from 'svelte-fa';
	import { faGaugeHigh, faRefresh, faTemperature0 } from '@fortawesome/free-solid-svg-icons';
	import ScrollArea from '$lib/components/ui/scroll-area/scroll-area.svelte';
	import Input from '$lib/components/ui/input/input.svelte';
	import type { RmdBrakeRequest, RmdCanIdResponse, RmdDataRequest, RMDStatusRequest, RMDStatusResponse, SuccessServiceResponse as SuccessResponse } from './rmdConfig/serviceTypes';
	import * as AlertDialog from '$lib/components/ui/alert-dialog';
	import Widget from '$lib/components/widget.svelte';

	const errorConstants: Record<number, string> = {
		0x0000: 'NO_ERROR',
		0x0002: 'STALL',
		0x0004: 'LOW_VOLTAGE',
		0x0008: 'OVER_VOLTAGE',
		0x0010: 'OVER_CURRENT',
		0x0040: 'POWER_OVERRUN',
		0x0080: 'CALIBRATION_PARAMETER_WRITE',
		0x0100: 'SPEEDING',
		0x1000: 'OVER_TEMPERATURE',
		0x2000: 'ENCODER_CALIBRATION'
	};

	const getErrorList = (errorCode: number): string[] => {
		const errors: string[] = [];
		for (const [code, description] of Object.entries(errorConstants)) {
			if ((errorCode & Number(code)) !== 0) {
				errors.push(description);
			}
		}
		return errors;
	};

	let enableStatusService: ROSLIB.Service<{ data: boolean }, { success: boolean }> | null = null;
	let statusService: ROSLIB.Service<RMDStatusRequest, RMDStatusResponse> | null = null;
	let canIdService: ROSLIB.Service<{}, RmdCanIdResponse> | null = null;
	let brakeEnableService: ROSLIB.Service<RmdBrakeRequest, SuccessResponse> | null = null;
	let restartMotorService: ROSLIB.Service<RmdDataRequest, SuccessResponse> | null = null;
	let setIdService: ROSLIB.Service<RmdDataRequest, SuccessResponse> | null = null;
	let zeroMotorService: ROSLIB.Service<RmdDataRequest, SuccessResponse> | null = null;

	let motorStatus = $state<RMDStatusResponse | null>(null);
	let ids = $state<Array<number>>([]);
	let selectedIdInput = $state<string>('0');
	let selectedId = $derived(Number(selectedIdInput));
	let debugEnabled = $state<boolean>(false);

	// Widget logic goes here
	$effect(() => {
		const ros = getRosConnection();

		if (ros) {
			statusService = new ROSLIB.Service({
				ros: ros,
				name: '/arm/rmd_status',
				serviceType: 'perseus_msgs/srv/RmdServoStatus'
			});

			canIdService = new ROSLIB.Service({
				ros: ros,
				name: '/arm/rmd_can_ids',
				serviceType: 'perseus_msgs/srv/RmdCanId'
			});

			brakeEnableService = new ROSLIB.Service({
				ros: ros,
				name: '/arm/rmd_set_brake',
				serviceType: 'perseus_msgs/srv/RmdBrake'
			});

			restartMotorService = new ROSLIB.Service({
				ros: ros,
				name: '/arm/rmd_restart_motor',
				serviceType: 'perseus_msgs/srv/RmdData'
			});

			enableStatusService = new ROSLIB.Service({
				ros: ros,
				name: '/arm/rmd_enable_status_messages',
				serviceType: 'std_srvs/srv/SetBool'
			});

			setIdService = new ROSLIB.Service({
				ros: ros,
				name: '/arm/rmd_set_motor_id',
				serviceType: 'perseus_msgs/srv/RmdData'
			});

			zeroMotorService = new ROSLIB.Service({
				ros: ros,
				name: '/arm/rmd_set_zero_position',
				serviceType: 'perseus_msgs/srv/RmdData'
			});

			callGetIds();
		} else {
			statusService = null;
			canIdService = null;
			brakeEnableService = null;
			restartMotorService = null;
			enableStatusService = null;
			setIdService = null;
			zeroMotorService = null;
		}
	});

	const getMotorStatus = () => {
		if (statusService && selectedId > 0) {
			const request: RMDStatusRequest = {
				motor_id: selectedId
			};

			statusService.callService(request, (response: RMDStatusResponse) => {
				motorStatus = response;
			});
		}
	};

	const callGetIds = () => {
		if (canIdService) {
			canIdService.callService({}, (response: RmdCanIdResponse) => {
				ids = [0, ...response.servo_ids];
			});
		}
	}

	const callBrakeEnable = (enable: boolean) => {
		if (brakeEnableService) {
			const request = {
				motor_id: selectedId,
				brake_enable: enable
			};

			brakeEnableService.callService(request, (response: { success: boolean }) => {
				console.log(`Brake ${enable ? 'enabled' : 'disabled'} for motor ID ${selectedId}: ${response.success}`);
			});
		}
	};

	const callDebugEnable = () => {
		if (enableStatusService) {
			const request = {
				data: !debugEnabled
			};

			enableStatusService.callService(request, () => {
				debugEnabled = !debugEnabled;
			});
		}
	};

	const restartMotor = () => {
		if (restartMotorService) {
			const request: RmdBrakeRequest = {
				motor_id: selectedId,
			};

			restartMotorService.callService(request, (response: { success: boolean }) => {
				console.log(`Restart command sent for motor ID ${selectedId}: ${response.success}`);
			});
		}
	};

	let zeroBias = $state<number>(0);
	const setMotorZero = (bias?: number) => {
		if (zeroMotorService) {
			const request: RmdDataRequest = {
				motor_id: selectedId,
				trigger: !bias, // true is settings zero to current position
				data: bias ?? 0 // Data field is unused for this service
			};

			if (bias) zeroBias = 0;

			zeroMotorService.callService(request, (response: { success: boolean }) => {
				console.log(`Zero position command sent for motor ID ${selectedId}: ${response.success}`);
			});
		}
	};

	let newMotorId = $state<number>(0);
	let setIdConfirm = $state<boolean>(false);
	// id can be between 1 and 32 inclusive
	const isMotorIdValid = $derived(newMotorId > 0 && newMotorId <= 32);
	const setMotorId = () => {
		if (isMotorIdValid && setIdService) {
			const request: RmdDataRequest = {
				motor_id: selectedId,
				data: newMotorId
			};

			setIdService.callService(request, (response: { success: boolean }) => {
				ids = [];
				selectedId = 0;
				callGetIds();
				setIdConfirm = false;
			});
		}
	}

	onMount(() => {
		const interval = setInterval(() => {
			getMotorStatus();
		}, 100);

		return () => clearInterval(interval);
	});
	
</script>

<div class="flex">
	<strong class="my-auto text-xl">Motor ID:</strong>
	<div class="mx-2 w-[60px]">
		<Select.Root type="single" bind:value={selectedIdInput}>
			<Select.Trigger>
				{selectedId === 0 ? 'All' : selectedId}
			</Select.Trigger>
			<Select.Content>
				{#each ids as id}
				<Select.Item value={id.toString()}>{id === 0 ? 'All' : id}</Select.Item>
				{/each}
			</Select.Content>
		</Select.Root>
	</div>
	<Button size="icon" variant="outline" onclick={callGetIds}><Fa icon={faRefresh} /></Button>
	<Button class="ml-auto" onclick={callDebugEnable}>{debugEnabled ? 'Disable ' : 'Enable '} Debug</Button>
</div>

<div class="flex gap-2 mt-2 h-full flex-wrap">
	<div class="min-w-[200px] mb-2 w-1/4">
		<div class="border aspect-square w-full rounded-2xl relative">
			{#if selectedId !== 0}
				<div class="text-center content-center absolute top-[50%] left-[50%] translate-x-[-50%] translate-y-[-50%] w-2/3 aspect-square">
					<p><strong>Temp</strong>: {motorStatus?.temperature.toFixed(2)}&deg;C</p>
					<p><strong>RPM</strong>: {motorStatus?.speed.toFixed(2)} dps</p>
					<p><strong>Volts</strong>: {motorStatus?.voltage.toFixed(2)} V</p>
					<p><strong>Amps</strong>: {motorStatus?.torque_current.toFixed(2)} A</p>
					<p><strong>Angle</strong>: {motorStatus?.angle.toFixed(2)}&deg;</p>
				</div>
				<div class="z-5 absolute aspect-square w-[80%] top-[10%] left-[10%] border rounded-[50%]">
					<span class="absolute border border-l-0 border-r left-1/2 h-[5%] translate-x-[-1px]"></span>
					<span class="absolute border border-l-0 border-r left-1/2 bottom-0 h-[5%] translate-x-[-1px]"></span>
					<span class="absolute border border-b-0 border-t top-1/2 w-[5%] translate-x-[-1px]"></span>
					<span class="absolute border border-b-0 border-t top-1/2 right-0 w-[5%] translate-x-[-1px]"></span>
				</div>
				<svg viewBox="0 0 100 100" class="w-full h-full absolute top-0 left-0 z-10">
					<circle 
						r="3%"
						cx={`${40 * Math.cos((motorStatus?.angle! * -1 - 90) * Math.PI / 180) + 50}%`}
						cy={`${40 * Math.sin((motorStatus?.angle! * -1 - 90) * Math.PI / 180) + 50}%`}
						fill="red"
					/>
				</svg>
			{:else}
				<div class="text-center content-center absolute top-[50%] left-[50%] translate-x-[-50%] translate-y-[-50%] w-1/2 aspect-square">
					<p>Select a single motor ID to view status</p>
				</div>
			{/if}
		</div>
	</div>
	<div class="w-1/2 min-w-[400px]">
		<div style:aspect-ratio="2 / 1" class="w-full border rounded-2xl p-2 flex flex-col relative">
			{#if selectedId !== 0}
				<p class="">Change Motor ID:</p>
				<div class="flex gap-2 mb-2">
					<Input bind:value={newMotorId} type="number" />
					<AlertDialog.Root bind:open={setIdConfirm}>
						<AlertDialog.Trigger>
							<Button disabled={!isMotorIdValid}>Set</Button>
						</AlertDialog.Trigger>
						<AlertDialog.Content>
							<AlertDialog.Header>
								<AlertDialog.Title>
									Caution: This will restart all connected motors
								</AlertDialog.Title>
								<AlertDialog.Description>
									This will disengage <strong>all</strong> motors breaks. You must ensure they are not under load.
								</AlertDialog.Description>
							</AlertDialog.Header>
							<AlertDialog.Footer>
								<AlertDialog.Cancel>Cancel</AlertDialog.Cancel>
								<AlertDialog.Action onclick={setMotorId}>Confirm</AlertDialog.Action>
							</AlertDialog.Footer>
						</AlertDialog.Content>
					</AlertDialog.Root>
				</div>
				<p class="">Set Zero Bias:</p>
				<div class="flex gap-2 mb-2">
					<Input bind:value={zeroBias} type="number" />
					<Button onclick={() => setMotorZero(zeroBias)}>Set</Button>
					<Button onclick={() => setMotorZero()}>Set Current as Zero</Button>
				</div>
				<Button disabled={selectedId === 0} variant="outline" onclick={restartMotor}>Restart</Button>
			{:else}
				<div class="text-center content-center absolute top-[50%] left-[50%] translate-x-[-50%] translate-y-[-50%] w-1/2 aspect-square">
					<p>Select a single motor ID to view status</p>
				</div>
			{/if}
		</div>
	</div>
	<div class="min-w-[200px] mb-2">
		<p><strong>Motor ID</strong>: {motorStatus?.motor_id}</p>
		<p><strong>Brake engaged</strong>: {motorStatus?.brake_control ? 'Yes' : 'No'}</p>
		<p><strong>Phase current</strong>:<br />a -> {motorStatus?.phase_a_current.toFixed(2)}, <br />b -> {motorStatus?.phase_b_current.toFixed(2)},<br />c -> {motorStatus?.phase_c_current.toFixed(2)}</p>
		<p><strong>Errors</strong>: {getErrorList(motorStatus?.error ?? 0).join(', ')}</p>
	</div>
</div>

<!-- <p>ids: {ids}</p>
<button onclick={callGetIds}>Get Ids</button>

<p>{output}</p>
<button onclick={callService}>Get RMD Status</button> -->
