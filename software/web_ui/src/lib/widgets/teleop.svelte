<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Joystick Teleop';
	export const description = 'Control the rover with a joystick';
	export const group = 'ROS';
	export const isRosDependent = true;

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			General: {
				joyStickRadius: {
					type: 'number',
					description: 'Joystick Radius in pixels',
					value: '64'
				},
				linearSpeedMultiplier: {
					type: 'number',
					description: 'Forwards and backwards speed multiplier for the joystick',
					value: '1'
				},
				angularSpeedMultiplier: {
					type: 'number',
					description: 'Left and right speed multiplier for the joystick',
					value: '0.5'
				}
			}
		}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import * as ROSLIB from 'roslib';
	import { onMount } from 'svelte';
	import Button from '$lib/components/ui/button/button.svelte';

	// wrapper for widget setting
	let joystickRadius = $derived<number>(Number(settings.groups.General.joyStickRadius.value));

	let topic: ROSLIB.Topic | null = null;
	let intervalHandle: NodeJS.Timeout | null = null;
	let lockingTimeoutHandle: NodeJS.Timeout | null = null;
	let joystickValue = { x: 0, y: 0 };
	let mousePosition = { x: 0, offsetX: 0, y: 0, offsetY: 0 };
	let unlocked = $state<boolean>(false);
	let data = $state<string>();
	let joystick = $state<{
		active: boolean;
		container: HTMLDivElement | null;
		origin: { x: number; y: number };
	}>({
		active: false,
		container: null,
		origin: { x: 0, y: 0 }
	});

	// HTML components for the joystick
	const joystickHandle = document.createElement('div');
	const joystickBase = document.createElement('div');
	joystickHandle.classList.add('bg-red-500', 'opacity-50', 'rounded-full', 'absolute', 'z-30');
	joystickBase.classList.add('bg-red-500', 'opacity-50', 'rounded-[10%]', 'absolute', 'z-20');

	const updateJoystick = (x: number, y: number) => {
		joystickHandle.style.left = `calc(${x}px - ${joystickRadius / 8}px)`;
		joystickHandle.style.top = `calc(${y}px - ${joystickRadius / 8}px)`;
		joystick.container?.appendChild(joystickHandle);
	};

	const hideJoystick = () => {
		joystickValue.x = 0;
		joystickValue.y = 0;
		joystick.active = false;

		let forRemove = [];

		for (const child of joystick.container?.children as HTMLCollection) {
			if (child.nodeName.toLowerCase() === 'div') forRemove.push(child);
		}

		for (const child of forRemove) {
			joystick.container?.removeChild(child);
		}
	};

	const deadbandAxis = (value: number, deadband: number) => {
		let clamped: boolean = value < deadband && value > -deadband;
		if (clamped) value = 0;
		else if (value > deadband) value -= deadband;
		else if (value < deadband) value += deadband;

		// re-normalise the axis
		if (!clamped) {
			if (value > 0) value /= 1 - deadband;
			else value /= 1 - deadband;
		}

		return Math.round(value * 1000) / 1000;
	};

	const onStart = (e: PointerEvent) => {
		if (!unlocked) return;

		// set the joystick size before displaying
		joystickBase.style.width = `${joystickRadius}px`;
		joystickBase.style.height = `${joystickRadius}px`;
		joystickHandle.style.width = `${joystickRadius / 4}px`;
		joystickHandle.style.height = `${joystickRadius / 4}px`;

		joystick.origin = { x: e.offsetX, y: e.offsetY };
		mousePosition = {
			x: e.offsetX,
			offsetX: e.clientX - e.offsetX,
			y: e.offsetY,
			offsetY: e.clientY - e.offsetY
		};

		// display the joystick
		joystick.active = true;
		joystickBase.style.left = `calc(${e.clientX}px - ${e.clientX - e.offsetX + joystickRadius / 2}px)`;
		joystickBase.style.top = `calc(${e.clientY}px - ${e.clientY - e.offsetY + joystickRadius / 2}px`;
		joystick.container?.appendChild(joystickBase);
		updateJoystick(mousePosition.x, mousePosition.y);

		// reset locking timeout
		if (lockingTimeoutHandle) {
			clearTimeout(lockingTimeoutHandle);
		}
		lockingTimeoutHandle = null;

		// debug
		data =
			`<p>joystick position: ${joystick.origin.x}, ${joystick.origin.y}.</br>` +
			`mouse position: ${mousePosition.x}, ${mousePosition.y}. mouse position offset: ${mousePosition.offsetX}, ${mousePosition.offsetY}.<p/>`;
	};

	const onMove = (e: PointerEvent) => {
		if (!joystick.active) return;

		// update mouse position
		mousePosition.x = e.clientX - mousePosition.offsetX;
		mousePosition.y = e.clientY - mousePosition.offsetY;

		// calculate the distance from the joystick origin
		let dx = mousePosition.x - joystick.origin.x;
		let dy = mousePosition.y - joystick.origin.y;
		let halfRadius = joystickRadius / 2;
		if (dx > halfRadius) dx = halfRadius;
		if (dx < -halfRadius) dx = -halfRadius;
		if (dy > halfRadius) dy = halfRadius;
		if (dy < -halfRadius) dy = -halfRadius;

		// normalise the joystick value
		joystickValue.x = dx / halfRadius;
		joystickValue.y = dy / halfRadius;

		// add dead bands
		const deadband = 0.2;
		joystickValue.x = deadbandAxis(joystickValue.x, deadband);
		joystickValue.y = deadbandAxis(joystickValue.y, deadband);

		// display the joystick
		updateJoystick(joystick.origin.x + dx, joystick.origin.y + dy);

		// debug
		data =
			`<p>joystick position: ${joystick.origin.x}, ${joystick.origin.y}.</br>` +
			`mouse position: ${mousePosition.x}, ${mousePosition.y}. mouse position offset: ${mousePosition.offsetX}, ${mousePosition.offsetY}.<p/>` +
			`<p>Thumb offsets (x: ${dx}, y: ${dy})</p>` +
			`<p>Offsets with limits: (x: ${joystickValue.x}, y: ${joystickValue.y})</p>`;
	};

	const onStop = () => {
		if (!joystick.active || !unlocked) return;
		hideJoystick();

		// send 0 message to terminate
		if (topic) {
			topic.publish({
				header: {}, // Leaving this empty forces ROS bridge to fill in the timestamp.
				twist: {
					linear: {
						x: 0,
						y: 0,
						z: 0
					},
					angular: {
						x: 0,
						y: 0,
						z: 0
					}
				}
			});
		}

		// lock the joystick
		lockingTimeoutHandle = setTimeout(() => {
			unlocked = false;
		}, 10000);
	};

	const initTeleop = () => {
		if (!joystick.container) return;

		joystick.container.addEventListener('pointerdown', onStart);
		joystick.container.addEventListener('pointermove', onMove);
		joystick.container.addEventListener('pointerup', onStop);
		joystick.container.addEventListener('pointerleave', onStop);

		intervalHandle = setInterval(() => {
			if (joystick.active && topic) {
				topic.publish({
					header: {}, // Leaving this empty forces ROS bridge to fill in the timestamp.
					twist: {
						linear: {
							x: -joystickValue.y * Number(settings.groups.General.linearSpeedMultiplier.value),
							y: 0,
							z: 0
						},
						angular: {
							x: 0,
							y: 0,
							z: -joystickValue.x * Number(settings.groups.General.angularSpeedMultiplier.value)
						}
					}
				});
			}
		}, 75);
	};

	onMount(() => {
		unlocked = false;

		return () => {
			if (intervalHandle) clearInterval(intervalHandle);
		};
	});

	// handle ros connection/disconnection
	$effect(() => {
		const ros = getRosConnection();
		if (ros) {
			topic = new ROSLIB.Topic({
				ros: ros,
				name: '/web_vel',
				messageType: 'geometry_msgs/TwistStamped'
			});
		} else {
			topic = null;
		}
	});

	let unlocking = false;
	const startUnlockTimer = () => {
		unlocking = true;
		setTimeout(() => {
			if (unlocking) {
				unlocked = true;
				unlocking = false;
				initTeleop();

				// lock the joystick
				lockingTimeoutHandle = setTimeout(() => {
					unlocked = false;
				}, 10000);
			}
		}, 1000);
	};

	const stopUnlockTimer = () => {
		unlocking = false;
		unlocked = false;
	};
</script>

<svelte:window onblur={stopUnlockTimer} />

<div
	bind:this={joystick.container}
	class="relative h-full w-full border bg-transparent"
	style:z-index="2"
>
	<!-- <p style:z-index="-1">{data}</p> -->
	<!-- {@html data} -->
</div>
<!-- Message must be on a lower z index so it doesn't trigger a mouse down event -->
{#if !joystick.active}
	<p
		class="absolute left-[50%] top-[50%] -translate-x-[50%] -translate-y-[50%] overflow-hidden text-xl"
		style:filter={unlocked ? 'none' : 'blur(12px)'}
		style:z-index="1"
	>
		Press and hold to start moving the joystick.
	</p>
{/if}

{#if !unlocked}
	<div
		class="absolute left-[50%] top-[50%] -translate-x-[50%] -translate-y-[50%] overflow-hidden text-xl"
		style:z-index="3"
	>
		<Button variant="outline" onpointerdown={startUnlockTimer} onpointerup={stopUnlockTimer}
			>Hold to unlock</Button
		>
	</div>
{/if}
