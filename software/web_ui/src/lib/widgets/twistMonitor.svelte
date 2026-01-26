<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Twistometer';
	export const description = 'Vector based gauge to display raw ros twist messages';
	export const group = 'ROS';
	export const isRosDependent = false;

	export const settings = $state<WidgetSettingsType>({
		groups: {
			General: {
				topic: {
					type: 'select',
					description: 'ROS topic to subscribe to',
					options: [],
					value: ''
				},
				MaxXValue: {
					type: 'number',
					description: 'Maximum value for the x axis',
					value: '2'
				},
				MaxYValue: {
					type: 'number',
					description: 'Maximum value for the y axis',
					value: '2'
				},
				MaxZValue: {
					type: 'number',
					description: 'Maximum value for the z axis',
					value: '2'
				}
			}
		}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte';
	import type { StampedTwistMessageType } from '$lib/scripts/rosTypes';
	import * as ROSLIB from 'roslib';
	import { onMount } from 'svelte';

	let canvas = $state<HTMLCanvasElement | null>(null);
	let context = $state<CanvasRenderingContext2D | null>(null);
	// alias and type conversion for settings.groups.General.gaugeSize.value
	let canvasSize = 600;
	let twist = $state<{ x: number; y: number; rotation: number }>({ x: 0, y: 0, rotation: 0 });

	function canvas_arrow(
		context: CanvasRenderingContext2D,
		fromx: number,
		fromy: number,
		tox: number,
		toy: number
	) {
		if (!context) return;
		var headlen = 10; // length of head in pixels
		var dx = tox - fromx;
		var dy = toy - fromy;
		var angle = Math.atan2(dy, dx);
		context.fillStyle = '#dc2626';
		context.strokeStyle = '#dc2626';
		context.lineCap = 'round';
		context.lineWidth = 4;
		context.moveTo(fromx, fromy);
		context.lineTo(tox, toy);
		context.moveTo(tox, toy);
		context.lineTo(
			tox - headlen * Math.cos(angle - Math.PI / 6),
			toy - headlen * Math.sin(angle - Math.PI / 6)
		);

		context.lineTo(
			tox - headlen * Math.cos(angle + Math.PI / 6),
			toy - headlen * Math.sin(angle + Math.PI / 6)
		);
		context.lineTo(tox, toy);
	}

	const drawArrow = (x: number, y: number) => {
		canvas_arrow(context!, canvasSize / 2, canvasSize / 2, canvasSize / 2 + y, canvasSize / 2 + x);
	};

	let twistTopic: ROSLIB.Topic<StampedTwistMessageType> | null = null;

	$effect(() => {
		if (getRosConnection()) {
			const ros = getRosConnection() as ROSLIB.Ros;
			if (settings.groups.General.topic.value !== '') {
				twistTopic = new ROSLIB.Topic({
					ros: ros,
					name: settings.groups.General.topic.value!,
					messageType: 'geometry_msgs/msg/TwistStamped'
				});

				twistTopic.subscribe((message: any) => {
					// Switch data so that controls are correct directions
					twist.y = message.twist.linear.x;
					twist.x = message.twist.linear.y;
					twist.rotation = -message.twist.angular.z;
				});
			}

			// only add twist topics to the dropdown
			ros.getTopics((topics) => {
				settings.groups.General.topic.options = [];
				for (let i = 0; i < topics.types.length; i++) {
					if (topics.types[i].includes('Twist')) {
						settings.groups.General.topic.options.push({
							value: topics.topics[i],
							label: topics.topics[i]
						});
					}
				}
			});
		}
	});

	$effect(() => {
		context?.clearRect(0, 0, canvasSize, canvasSize);
		context?.beginPath();
		drawArrow(
			(twist.x / Number(settings.groups.General.MaxXValue.value)) * (canvasSize / 2),
			(twist.y / Number(settings.groups.General.MaxYValue.value)) * (canvasSize / 2)
		);
		context?.stroke();
	})

	onMount(() => {
		if (canvas) {
			context = canvas.getContext('2d');

			context?.setTransform(1, 0, 0, 1, 0, 0);
			context?.translate(canvasSize / 2, canvasSize / 2);
			context?.rotate(-Math.PI / 2);
			context?.translate(-canvasSize / 2, -canvasSize / 2);
		}

		
			//generate random numbers for testing
			twist.x = (Math.random() - 0.5) * 4;
			twist.y = (Math.random() - 0.5) * 4;
			twist.rotation = (Math.random() - 0.5) * 4
		

		return () => {
			if (twistTopic) twistTopic.unsubscribe();
		};
	});

	const RADIUS = 45;
</script>

<div class="flex flex-col w-full h-full"
		style:--transition-length="100ms">
	<div class="flex-1">
		<div class="aspect-square max-h-full max-w-full p-[8%] relative">
			<svg fill="none" class=" z-2 absolute top-0 left-0" stroke-width="2" viewBox="0 0 100 100">
				<circle
					r={RADIUS}
					stroke="#dc26263f"
					style:--stroke-percent={100 / 2}
					style:--circumference={2 * Math.PI * RADIUS}
				/>
				<circle
					r={RADIUS}
					stroke="#dc2626"
					style:transform="rotate(-90deg) scale(1, -1)"
					style:--stroke-percent={-(
						(twist.rotation / Number(settings.groups.General.MaxZValue.value)) *
						25
					)}
					style:--circumference={2 * Math.PI * RADIUS}
				/>
				<circle
					r={RADIUS}
					stroke="#dc2626"
					style:transform="rotate(-90deg)"
					style:--stroke-percent={(twist.rotation / Number(settings.groups.General.MaxZValue.value)) *
						25}
					style:--circumference={2 * Math.PI * RADIUS}
				/>
			</svg>
			<div class=" rounded-full relative w-full h-full border">
				<span class="absolute border border-l-0 border-r left-1/2 top-0 h-[5%] translate-x-[-1px]"></span>
				<p class="absolute left-[calc(50%-0.25em)] top-0 mt-[5%] translate-x-[-1px]">{settings.groups.General.MaxYValue.value}</p>
				<span class="absolute border border-l-0 border-r left-1/2 bottom-0 h-[5%] translate-x-[-1px]"></span>
				<p class="absolute left-[calc(50%-0.25em)] bottom-0 mb-[5%] translate-x-[-1px]">-{settings.groups.General.MaxYValue.value}</p>
				<span class="absolute border border-b-0 border-t top-1/2 w-[5%] translate-x-[-1px]"></span>
				<p class="absolute top-[calc(50%-0.6em)] left-0 ml-[6%] translate-x-[-1px]">-{settings.groups.General.MaxXValue.value}</p>
				<span class="absolute border border-b-0 border-t top-1/2 right-0 w-[5%] translate-x-[-1px]"></span>
				<p class="absolute top-[calc(50%-0.6em)] right-0 mr-[6%] translate-x-[-1px]">{settings.groups.General.MaxXValue.value}</p>
				<canvas
					bind:this={canvas}
					width={canvasSize}
					height={canvasSize}
					class="absolute left-0 top-0 h-full w-full"
				></canvas>					
			</div>
		</div>
	</div>
	<p class="mx-auto">Angular -> z: {Math.round(twist.rotation * 100) / 100}</p>
	<p class="mx-auto">Linear -> x: {Math.round(twist.x * 100) / 100}, y: {Math.round(twist.y * 100) / 100}</p>
</div>

<!-- <div class="flex w-fit flex-col items-center justify-center gap-2">
	<div
		class="relative w-fit overflow-hidden rounded-[50%]"
		style:--circle-size="100px"
		style:--transition-length="100ms"
	>
		<svg fill="none" class="z-2 absolute size-full" stroke-width="2" viewBox="0 0 100 100">
			<circle
				r="42"
				stroke="#dc26263f"
				style:--stroke-percent={100 / 2}
				style:--circumference={2 * Math.PI * 42}
			/>
			<circle
				r="42"
				stroke="#dc2626"
				style:transform="rotate(-90deg) scale(1, -1)"
				style:--stroke-percent={-(
					(twist.rotation / Number(settings.groups.General.MaxZValue.value)) *
					25
				)}
				style:--circumference={2 * Math.PI * 42}
			/>
			<circle
				r="42"
				stroke="#dc2626"
				style:transform="rotate(-90deg)"
				style:--stroke-percent={(twist.rotation / Number(settings.groups.General.MaxZValue.value)) *
					25}
				style:--circumference={2 * Math.PI * 42}
			/>
		</svg>
		<div
			class="border-card-foreground relative m-[40px] w-fit rounded-[50%] border"
			style:width={`${canvasSize}px`}
			style:height={`${canvasSize}px`}
		>
			<canvas bind:this={canvas} width={canvasSize} height={canvasSize} class="absolute"></canvas>
			<div class="absolute left-0 top-0 z-0 h-full w-full">
				<div class="mt-[calc(50%-60px)] flex h-[60px] border-b">
					<p class="ml-2 mt-auto">-{settings.groups.General.MaxXValue.value}</p>
					<p class="ml-auto mr-2 mt-auto">{settings.groups.General.MaxXValue.value}</p>
				</div>
			</div>
			<div class="absolute left-0 top-0 z-0 h-full w-full">
				<div class="ml-[50%] flex h-full flex-col border-l">
					<p class="ml-2 mt-2">{settings.groups.General.MaxYValue.value}</p>
					<p class="mb-2 ml-2 mt-auto">-{settings.groups.General.MaxYValue.value}</p>
				</div>
			</div>
		</div>
	</div>
	<div class="-mt-[40px] flex flex-row gap-2">
		<div>
			<p>Angular:</p>
			<p>z: {Math.round(twist.rotation * 100) / 100}</p>
		</div>
		<div>
			<p>Linear:</p>
			<p>x: {Math.round(twist.x * 100) / 100}, y: {Math.round(twist.y * 100) / 100}</p>
		</div>
	</div>
</div>
 -->
<style>
	canvas {
		z-index: 1;
	}

	circle {
		cx: 50;
		cy: 50;
		stroke-linecap: round;
		stroke-linejoin: round;

		transform: scale(-1, -1);
		stroke-dasharray: calc(var(--stroke-percent) * calc(var(--circumference) / 100))
			var(--circumference);
		transition: all var(--transition-length) ease 0s;
		transform-origin: calc(var(--circle-size) / 2) calc(var(--circle-size) / 2);
	}
</style>
