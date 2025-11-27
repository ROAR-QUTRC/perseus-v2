<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Bucket Controller';
	// These properties are optional
	export const description = 'Control the E&C bucket via ROS';
	export const group: WidgetGroupType = 'ROS';
	export const isRosDependent = true; // Set to true if the widget requires a ROS connection

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			general: {
				masterSpeedMultiplier: {
					type: 'number',
					description: 'Base multiplier applied to all speeds',
					value: '1'
				},
				lift: {
					type: 'number',
					description: 'Multiplier for lift speed',
					value: '0.1'
				},
				tilt: {
					type: 'number',
					description: 'Multiplier for tilt speed',
					value: '0.1'
				},
				jaws: {
					type: 'number',
					description: 'Multiplier for jaws speed',
					value: '0.1'
				},
				rotate: {
					type: 'number',
					description: 'Multiplier for rotate speed',
					value: '0.5'
				}
			}
		}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/ros-bridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import ROSLIB from 'roslib';
	import { onMount } from 'svelte';
	import Button from '$lib/components/ui/button/button.svelte';

	const CALLBACK_INTERVAL = 100; // 10Hz
	const INACTIVE_TIMEOUT = 10000; // 10 seconds

	let intervalHandle: NodeJS.Timeout | null = null;
	let lockingTimeoutHandle: NodeJS.Timeout | null = null;
	let topic: ROSLIB.Topic | null = null;
	let magnet = $state<boolean>(false);
	const handles = $state<
		Record<string, { parent: HTMLDivElement | null; active: boolean; value: number }>
	>({
		lift: { parent: null, active: false, value: 0 },
		tilt: { parent: null, active: false, value: 0 },
		jaws: { parent: null, active: false, value: 0 },
		rotate: { parent: null, active: false, value: 0 }
	});

	// ROS2 connection management
	$effect(() => {
		const ros = getRosConnection();

		if (ros) {
			topic = new ROSLIB.Topic({
				ros: ros,
				name: '/bucket_actuators',
				messageType: 'actuator_msgs/Actuators'
			});
			topic.advertise();
		} else {
			topic = null;
		}
	});

	const deadBand = (value: number, threshold: number) => {
		return Math.abs(value) < threshold ? 0 : value;
	};

	const publishActuatorValues = () => {
		if (!topic) return;

		const velocities = [
			deadBand(handles.lift.value, 0.05) *
				Number(settings.groups.general.lift.value) *
				Number(settings.groups.general.masterSpeedMultiplier.value),
			deadBand(handles.tilt.value, 0.05) *
				Number(settings.groups.general.tilt.value) *
				Number(settings.groups.general.masterSpeedMultiplier.value),
			deadBand(handles.jaws.value, 0.05) *
				Number(settings.groups.general.jaws.value) *
				Number(settings.groups.general.masterSpeedMultiplier.value),
			deadBand(handles.rotate.value, 0.05) *
				Number(settings.groups.general.rotate.value) *
				Number(settings.groups.general.masterSpeedMultiplier.value)
		];

		const message = new ROSLIB.Message({
			header: {}, // Leaving this empty forces ROS bridge to fill in the timestamp.
			velocity: velocities,
			normalized: [magnet ? 1 : 0]
		});

		topic.publish(message);
	};

	// Unlock logic
	let unlocked = $state(false);
	let unlocking = false;

	const startLockingTimer = () => {
		return setTimeout(() => {
			Object.keys(handles).forEach((handle) => {
				onStop(handle);
			});
			publishActuatorValues();
			if (intervalHandle) clearInterval(intervalHandle);
			unlocked = false;
		}, INACTIVE_TIMEOUT);
	};

	const startUnlockTimer = () => {
		unlocking = true;
		setTimeout(() => {
			if (unlocking) {
				unlocked = true;
				unlocking = false;
				initSliders();

				// lock the sliders
				lockingTimeoutHandle = startLockingTimer();
			}
		}, 1000);
	};

	const stopUnlockTimer = () => {
		unlocking = false;
		unlocked = false;
	};

	// Joystick HTML components
	const move = (handle: string, clientY: number) => {
		const boundingRect = handles[handle].parent?.getBoundingClientRect();
		if (!boundingRect) return;
		const clampedY = Math.min(Math.max(clientY, boundingRect.top), boundingRect.bottom);
		handles[handle].value = -(((clampedY - boundingRect.top) / boundingRect.height) * 2 - 1);
	};

	const onStart = (event: PointerEvent, handle: string) => {
		event.preventDefault();

		handles[handle].active = true;
		move(handle, event.clientY);
		console.log('Start', handle, event.clientY);

		// reset locking timeout
		console.log('Resetting locking timeout', lockingTimeoutHandle);
		if (lockingTimeoutHandle) {
			clearTimeout(lockingTimeoutHandle);
		}
		lockingTimeoutHandle = null;
	};

	const onMove = (event: PointerEvent, handle: string) => {
		if (!handles[handle].active) return;

		event.preventDefault();

		move(handle, event.clientY);
	};

	const onStop = (handle: string, event?: PointerEvent) => {
		event?.preventDefault();

		handles[handle].value = 0;
		handles[handle].active = false;
		console.log('Stop', handle, event?.clientY);

		// start locking timeout
		const allInactive = Object.keys(handles).every((h) => !handles[h].active);
		if (allInactive && !lockingTimeoutHandle) {
			lockingTimeoutHandle = startLockingTimer();
		}
	};

	const initSliders = () => {
		setTimeout(() => {
			Object.keys(handles).forEach((handle) => {
				if (handles[handle].parent) {
					handles[handle].parent?.addEventListener('pointerdown', (e) => onStart(e, handle));
					handles[handle].parent?.addEventListener('pointermove', (e) => onMove(e, handle));
					handles[handle].parent?.addEventListener('pointerup', (e) => onStop(handle, e));
					handles[handle].parent?.addEventListener('pointerleave', (e) => onStop(handle, e));
					const rect = handles[handle].parent.getBoundingClientRect();
					console.log('Initialised: ', rect);
				} else console.warn('Failed to initialise: ', handle);
			});
		});

		intervalHandle = setInterval(() => {
			publishActuatorValues();
		}, CALLBACK_INTERVAL);
	};

	onMount(() => {
		unlocked = false;

		// Cleanup ros connection on unmount
		return () => {
			if (intervalHandle) clearInterval(intervalHandle);
		};
	});
</script>

{#if !unlocked}
	<div
		class="absolute left-[50%] top-[50%] -translate-x-[50%] -translate-y-[50%] overflow-hidden text-xl"
		style:z-index="3"
	>
		<Button variant="outline" onpointerdown={startUnlockTimer} onpointerup={stopUnlockTimer}>
			Hold to unlock
		</Button>
	</div>
{:else}
	<!-- Value is the height from -1 to 1 -->
	{#snippet thumb(active: boolean, value: number)}
		{#if active}
			<div
				style:top={`calc(${50 - value * 50}% - 25px)`}
				class={'absolute left-[5px] z-50 h-[50px] w-[50px] rounded-[25px] bg-blue-500'}
			></div>
		{/if}
	{/snippet}

	<div class=" flex h-full w-full flex-col">
		<div class="flex flex-1 gap-2">
			{#each Object.keys(handles) as key}
				<div class="flex flex-1 flex-col items-center justify-center gap-2 p-2">
					<p class="text-center" style="text-transform: capitalize;">{key}</p>
					<div
						bind:this={handles[key].parent}
						class=" relative z-40 h-[60%] w-[60px] rounded-[30px] bg-red-500 opacity-50"
					>
						{@render thumb(handles[key].active, handles[key].value)}
					</div>
				</div>
			{/each}
		</div>
		<Button class="w-fit self-center" onclick={() => (magnet = !magnet)}
			>Turn magnet {magnet ? 'off' : 'on'}</Button
		>
	</div>
{/if}
