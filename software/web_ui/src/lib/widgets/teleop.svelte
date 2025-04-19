<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';
	import { onMount, untrack } from 'svelte';

	export const name = 'Joystick Teleop - Under development';
	export const description = 'Control the rover with a joystick';
	export const group = 'ROS';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			General: {
				joyStickRadius: {
					type: 'number',
					description: 'Joystick Radius in pixels',
					value: '64'
				},
				speedMultiplyer: {
					type: 'number',
					description: 'Speed multiplyer for the joystick',
					value: '1'
				}
			}
		}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/ros-bridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import ROSLIB from 'roslib';

	let joystickRadius = $derived<number>(Number(settings.groups.General.joyStickRadius.value));

	let data = $state();
	let joystickActive = $state<boolean>(false);
	let joystickContainer = $state<HTMLDivElement | null>(null);

	const joystickHandle = document.createElement('div');
	joystickHandle.classList.add('bg-red-500', 'opacity-50', 'rounded-full', 'absolute', 'z-30');
	const joystickBase = document.createElement('div');
	joystickBase.classList.add('bg-red-500', 'opacity-50', 'rounded-full', 'absolute', 'z-20');

	let joystickOrigin = { x: 0, y: 0 };
	let mousePosition = { x: 0, offsetX: 0, y: 0, offsetY: 0 };
	let output = { x: 0, y: 0 };

	let topic: ROSLIB.Topic;

	const updateJoystick = (x: number, y: number) => {
		joystickHandle.style.left = `calc(${x}px - ${joystickRadius / 8}px)`;
		joystickHandle.style.top = `calc(${y}px - ${joystickRadius / 8}px)`;
		joystickContainer?.appendChild(joystickHandle);
	};

	const onStart = (e: PointerEvent) => {
		// set joy stick size
		joystickBase.style.width = `${joystickRadius}px`;
		joystickBase.style.height = `${joystickRadius}px`;
		joystickHandle.style.width = `${joystickRadius / 4}px`;
		joystickHandle.style.height = `${joystickRadius / 4}px`;

		joystickActive = true;
		joystickOrigin = { x: e.offsetX, y: e.offsetY };
		mousePosition = {
			x: e.offsetX,
			offsetX: e.clientX - e.offsetX,
			y: e.offsetY,
			offsetY: e.clientY - e.offsetY
		};

		// set initial positions
		updateJoystick(mousePosition.x, mousePosition.y);

		// create base
		joystickBase.style.left = `calc(${e.clientX}px - ${e.clientX - e.offsetX + joystickRadius / 2}px)`;
		joystickBase.style.top = `calc(${e.clientY}px - ${e.clientY - e.offsetY + joystickRadius / 2}px`;
		joystickContainer?.appendChild(joystickBase);

		// debug
		data =
			`<p>joystick position: ${joystickOrigin.x}, ${joystickOrigin.y}.</br>` +
			`mouse position: ${mousePosition.x}, ${mousePosition.y}. mouse position offset: ${mousePosition.offsetX}, ${mousePosition.offsetY}.<p/>`;
	};

	const onMove = (e: PointerEvent) => {
		if (joystickActive) {
			// update mouse position
			mousePosition.x = e.clientX - mousePosition.offsetX;
			mousePosition.y = e.clientY - mousePosition.offsetY;

			let dx = mousePosition.x - joystickOrigin.x;
			let dy = mousePosition.y - joystickOrigin.y;
			let x = mousePosition.x;
			let y = mousePosition.y;
			let magnitude = Math.sqrt(dx * dx + dy * dy);
			let limit = false;
			if (magnitude > joystickRadius / 2) {
				limit = true;
				let angle = Math.atan2(dy, dx);
				x = joystickOrigin.x + (joystickRadius / 2) * Math.cos(angle);
				y = joystickOrigin.y + (joystickRadius / 2) * Math.sin(angle);
			}
			// calculate bounds
			updateJoystick(x, y);

			output.x =
				((Math.round(x) - joystickOrigin.x) /
					Number(settings.groups.General.joyStickRadius.value!)) *
				2;
			output.y =
				(-(Math.round(y) - joystickOrigin.y) /
					Number(settings.groups.General.joyStickRadius.value!)) *
				2;
			data =
				`<p>joystick position: ${joystickOrigin.x}, ${joystickOrigin.y}.</br>` +
				`mouse position: ${mousePosition.x}, ${mousePosition.y}. mouse position offset: ${mousePosition.offsetX}, ${mousePosition.offsetY}.<p/>` +
				`<p>Magnitude: ${Math.round(magnitude)} -> should limit: ${limit}</br>(x, y) -> (${Math.round(x)}, ${Math.round(y)})</p>` +
				`<p>Translate to ros: ${output.x} ${output.y} (rel pos)</p>`;
		}
	};

	const onStop = () => {
		output.x = 0;
		output.y = 0;
		joystickActive = false;
		if (joystickContainer?.children.length! > 0) {
			joystickContainer?.removeChild(joystickHandle);
			joystickContainer?.removeChild(joystickBase);
		}
	};

	let intervalHandle: NodeJS.Timeout;
	onMount(() => {
		if (!joystickContainer) return;

		joystickContainer.addEventListener('pointerdown', onStart);
		joystickContainer.addEventListener('pointermove', onMove);
		joystickContainer.addEventListener('pointerup', onStop);

		intervalHandle = setInterval(() => {
			// send ros message every 75ms to prevent flooding the network
			let multiplyer = Number(settings.groups.General.speedMultiplyer.value);
			topic.publish({
				header: {}, // Leaving this empty forces ROS bridge to fill in the timestamp.
				twist: {
					linear: {
						x: output.y * multiplyer,
						y: 0,
						z: 0
					},
					angular: {
						x: 0,
						y: 0,
						z: output.x * multiplyer
					}
				}
			});
		}, 75);

		if (getRosConnection()) {
			topic = new ROSLIB.Topic({
				ros: getRosConnection() as ROSLIB.Ros,
				name: 'web_vel',
				messageType: 'geometry_msgs/TwistStamped'
			});
		}

		return () => {
			joystickContainer?.removeEventListener('pointerdown', onStart);
			joystickContainer?.removeEventListener('pointermove', onMove);
			joystickContainer?.removeEventListener('pointerup', onStop);
			clearInterval(intervalHandle);
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
		};
	});

	$effect(() => {
		let multiplyer = Number(settings.groups.General.speedMultiplyer.value);
		untrack(() => {
			if (multiplyer < 0) settings.groups.General.speedMultiplyer.value = '0';
			if (multiplyer > 2.5) settings.groups.General.speedMultiplyer.value = '2.5';
		});
	});
</script>

<div
	bind:this={joystickContainer}
	class="relative h-full w-full border bg-transparent"
	style:z-index="2"
>
	<!-- <p style:z-index="-1">{data}</p> -->
	<!-- {@html data} -->
</div>
<!-- Message must be on a lower z index so it doesn't trigger a mouse down event -->
{#if !joystickActive}
	<p
		class=" absolute left-[50%] top-[50%] -translate-x-[50%] -translate-y-[50%] text-xl"
		style:z-index="1"
	>
		Press and hold to start moving the joystick.
	</p>
{/if}
