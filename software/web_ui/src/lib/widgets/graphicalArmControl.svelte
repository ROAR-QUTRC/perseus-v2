<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Armed and Ready for Battle!';
	// These properties are optional
	export const description = 'Controll the arm joint angles with a graphical interface (NO IK)';
	export const group: WidgetGroupType = 'ROS';
	export const isRosDependent = true; // Set to true if the widget requires a ROS connection

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			outputs: {
				enableManualSend: {
					type: 'switch',
					value: 'false'
				}
			}
		}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import * as ROSLIB from 'roslib';
	import type { NumberArrayType } from '$lib/scripts/rosTypes';
	import Slider from '$lib/components/ui/slider/slider.svelte';
	import Button from '$lib/components/ui/button/button.svelte';

	const colors = { primary: '#ea580c', background: '#00ff00' };

	type JointNameType = 'SHOULDER_PAN' | 'SHOULDER_TILT' | 'ELBOW' | 'WRIST_PAN' | 'WRIST_TILT';
	interface JointStateType {
		joint: JointNameType;
		index: number;
		length: number;
		current: {
			angle: number;
			position: {
				x1: number;
				y1: number;
				x2: number;
				y2: number;
			};
		};
		target: {
			angle: number;
			position: {
				x1: number;
				y1: number;
				x2: number;
				y2: number;
			};
		};
	}

	// ROS logic

	let armAngleTopic: ROSLIB.Topic<NumberArrayType> | null = null;
	let armControlTopic: ROSLIB.Topic<NumberArrayType> | null = null;

	$effect(() => {
		const ros = getRosConnection();
		if (ros) {
			armAngleTopic = new ROSLIB.Topic({
				ros,
				name: '/arm/angles',
				messageType: 'std_msgs/Float64MultiArray'
			});
			armControlTopic = new ROSLIB.Topic({
				ros,
				name: '/arm/control',
				messageType: 'std_msgs/Float64MultiArray'
			});

			armAngleTopic.subscribe((message) => {
				const angles = message.data;
				if (angles.length === 5) {
					angles.forEach((angle, index) => {});
				}
			});
		}
	});

	// UI logic

	const LIVE_MESSAGE = false;

	const JOINT_ID_INDEX_MAP: Record<JointNameType, number> = {
		SHOULDER_PAN: 0,
		SHOULDER_TILT: 1,
		ELBOW: 2,
		WRIST_PAN: 3,
		WRIST_TILT: 4
	};

	// these are ordered accoring to the index in the JONT_ID_INDEX_MAP
	let angles = $state<
		Array<{
			jointId: JointNameType;
			angle: number;
			targetAngle: number;
			length: number | null;
			limits: { min: number; max: number };
			turnCount: number;
		}>
	>([
		{
			jointId: 'SHOULDER_PAN',
			angle: 0,
			targetAngle: 0,
			length: null,
			limits: { min: -90, max: 90 },
			turnCount: 0
		},
		{
			jointId: 'SHOULDER_TILT',
			angle: 0,
			targetAngle: 0,
			length: 25,
			limits: { min: 0, max: 180 },
			turnCount: 0
		},
		{
			jointId: 'ELBOW',
			angle: 0,
			targetAngle: 0,
			length: 40,
			limits: { min: 0, max: 180 },
			turnCount: 0
		},
		{
			jointId: 'WRIST_PAN',
			angle: 0,
			targetAngle: 0,
			length: null,
			limits: { min: -180, max: 180 },
			turnCount: 0
		},
		{
			jointId: 'WRIST_TILT',
			angle: 0,
			targetAngle: 0,
			length: 10,
			limits: { min: -180, max: 180 },
			turnCount: 0
		}
	]);

	let svgData = $derived.by<Array<JointStateType>>(() => {
		let totalLength = 0;
		const lastPos = { x: 50, y: 50 };
		const lastTargetPos = { x: 50, y: 50 };
		let lastAngle = 0;
		let lastTargetAngle = 0;
		return angles
			.filter((a) => a.length !== null)
			.map((point) => {
				totalLength += point.length!;
				return {
					joint: point.jointId,
					index: JOINT_ID_INDEX_MAP[point.jointId],
					angle: point.angle,
					targetAngle: point.targetAngle,
					length: point.length!
				};
			})
			.map((point) => {
				const angleRad = (point.angle + lastAngle) * (Math.PI / 180);
				const targetAngleRad = (point.targetAngle + lastTargetAngle) * (Math.PI / 180);
				const length = (point.length / totalLength) * 40; // scale to fit in the SVG
				lastAngle += point.angle;
				lastTargetAngle += point.targetAngle;
				const data: JointStateType = {
					joint: point.joint,
					index: point.index,
					length,
					current: {
						angle: point.angle,
						position: {
							x1: lastPos.x,
							y1: lastPos.y,
							x2: lastPos.x + Math.cos(angleRad) * length,
							y2: lastPos.y - Math.sin(angleRad) * length
						}
					},
					target: {
						angle: point.targetAngle,
						position: {
							x1: lastTargetPos.x,
							y1: lastTargetPos.y,
							x2: lastTargetPos.x + Math.cos(targetAngleRad) * length,
							y2: lastTargetPos.y - Math.sin(targetAngleRad) * length
						}
					}
				};
				lastPos.x = data.current.position.x2;
				lastPos.y = data.current.position.y2;
				lastTargetPos.x = data.target.position.x2;
				lastTargetPos.y = data.target.position.y2;
				return data;
			});
	});

	// handle dragging
	let selectedJoint = $state<JointStateType | null>(null);
	let relativeMousePos = $state<{ x: number; y: number } | null>(null);
	let selectedJointCenter: { x: number; y: number } | null = null;
	let angleOffset: number | null = null;
	let svgRect: { rect: DOMRect; xOffset: number; yOffset: number } | null = null;

	const showGhost = (e: MouseEvent, joint: JointStateType) => {
		const svgElement = (e.target as HTMLElement).closest('svg');

		if (!svgElement) return; // this shouldnt be possible
		const clientRect = svgElement.getClientRects()[0];
		svgRect = {
			rect: clientRect,
			xOffset: clientRect.width / 2,
			yOffset: clientRect.height / 2
		};
		selectedJoint = joint;
		relativeMousePos = {
			x: e.x - svgRect.rect.x - svgRect.xOffset,
			y: -(e.y - svgRect.rect.y - svgRect.yOffset)
		};
		selectedJointCenter = {
			x: (selectedJoint.target.position.x1 / 100) * svgRect.rect.width - svgRect.xOffset,
			y: -((selectedJoint.target.position.y1 / 100) * svgRect.rect.height - svgRect.yOffset)
		};
		angleOffset = 0;
		angles.forEach((angle, index) => {
			if (angleOffset !== null && angle.length && index < selectedJoint!.index) {
				const unwrappedAngle = angle.targetAngle % 360;
				angleOffset += unwrappedAngle;
				console.log('found some badness', unwrappedAngle);
			}
		});

		// console.log('final offset', angleOffset, angleOffset % 360);
		angleOffset = angleOffset % 360;

		// angleOffset =
		// 	angleOffset +
		// 	(angles[selectedJoint.index].turnCount > 0 ? -1 : 1) *
		// 		angles[selectedJoint.index].turnCount *
		// 		360;
		// if (Math.abs(angleOffset) > 360) console.log('badness');
	};

	let lastAngle: number | null = null;
	let currentAngle: number | null = null;
	const dragGhost = (e: MouseEvent) => {
		if (selectedJoint && svgRect && selectedJointCenter && angleOffset !== null) {
			relativeMousePos = {
				x: e.pageX - svgRect.rect.x - svgRect.xOffset,
				y: -(e.pageY - svgRect.rect.y - svgRect.yOffset)
			};

			const temp =
				Math.atan2(
					relativeMousePos.y - selectedJointCenter.y,
					relativeMousePos.x - selectedJointCenter.x
				) *
				(180 / Math.PI);
			const theta = temp < 0 ? temp : temp; // remap the output of atan2 to 0 to 360

			console.log('theta', theta, 'offset', angleOffset);
			lastAngle = currentAngle ?? theta - angleOffset;
			currentAngle = theta - angleOffset;

			// track rotations
			if (Math.abs(lastAngle - currentAngle) > 180) {
				if (lastAngle - currentAngle < 0) {
					angles[selectedJoint.index].turnCount--;
				} else {
					angles[selectedJoint.index].turnCount++;
				}
			}

			const draftAngle = theta - angleOffset + 360 * angles[selectedJoint.index].turnCount;
			angles[selectedJoint.index].targetAngle = Math.max(-180, Math.min(180, draftAngle));

			if (LIVE_MESSAGE) armControlTopic!.publish({ data: angles.map((a) => a.targetAngle) });
		}
	};

	const sendTargets = (forceSend: boolean = false) => {
		// only send command when a joint was selected
		if (
			armControlTopic &&
			((selectedJoint && svgRect && selectedJointCenter && angleOffset !== null) || forceSend)
		) {
			console.log(
				'Publishing angles:',
				angles.map((a) => a.targetAngle)
			);
			armControlTopic.publish({ data: angles.map((a) => a.targetAngle) });
		}

		selectedJoint = null;
		svgRect = null;
		selectedJointCenter = null;
		relativeMousePos = null;
		angleOffset = null;
		lastAngle = null;
		currentAngle = null;
	};
</script>

<div class="flex h-full flex-row">
	<div class="aspect-square max-h-full rounded-lg border">
		<!-- svelte-ignore a11y_no_static_element_interactions -->
		<!-- svelte-ignore a11y_click_events_have_key_events -->
		<svg
			viewBox="0 0 100 100"
			onmouseup={() => sendTargets()}
			onmouseleave={() => sendTargets()}
			onmousemove={dragGhost}
		>
			<!-- thick arm for making clicking easy -->
			{#each svgData as joint}
				<line
					onmousedown={(e) => showGhost(e, joint)}
					class="z-1 cursor-pointer"
					x1={joint.target.position.x1}
					y1={joint.target.position.y1}
					x2={joint.target.position.x2}
					y2={joint.target.position.y2}
					stroke-width="4"
					stroke={colors.background}
				/>
			{/each}
			{#each svgData as joint}
				<!-- visible arm -->
				<circle
					class="z-4 pointer-events-none"
					fill={colors.primary}
					cx={joint.current.position.x1}
					cy={joint.current.position.y1}
					r="1"
				/>
				<line
					class="z-4 pointer-events-none"
					x1={joint.current.position.x1}
					y1={joint.current.position.y1}
					x2={joint.current.position.x2}
					y2={joint.current.position.y2}
					stroke={colors.primary}
					stroke-width=".5"
				/>
				<!-- Ghost arm  -->
				<circle
					onmousedown={(e) => showGhost(e, joint)}
					class="z-3 cursor-pointer"
					fill={colors.primary}
					fill-opacity="0.2"
					cx={joint.target.position.x1}
					cy={joint.target.position.y1}
					r="1"
				/>
				<line
					class="z-3 pointer-events-none"
					x1={joint.target.position.x1}
					y1={joint.target.position.y1}
					x2={joint.target.position.x2}
					y2={joint.target.position.y2}
					stroke={colors.primary}
					stroke-opacity="0.2"
					stroke-width=".5"
				/>
				<text
					x={joint.current.position.x1 - 1}
					y={joint.current.position.y1 - 1.5}
					font-size="2"
					fill="white"
					class="z-5 pointer-events-none"
				>
					{joint.current.angle}&deg;
				</text>
				<text
					x={joint.current.position.x1 - 0.6}
					y={joint.current.position.y1 + 0.5}
					font-size="2"
					fill="black"
					class="z-5 pointer-events-none"
				>
					{joint.index}
				</text>
			{/each}
		</svg>
	</div>
	<div class=" flex-1 p-2">
		<div class="mb-2">
			<Button>Send Angles</Button>
		</div>
		<div class="h-fit w-full flex-1 rounded-lg border px-4 py-2">
			<p class="mb-2 text-xl"><strong>Shoulder Pan</strong></p>
			<Slider
				type="single"
				min={-90}
				max={90}
				bind:value={angles[JOINT_ID_INDEX_MAP.SHOULDER_PAN].targetAngle}
				onValueCommit={() => sendTargets(true)}
			/>
			<div class="flex flex-row gap-2">
				<div class="rounded-1/2 mb-4 mt-3 h-[8px] flex-1 rounded-[4px] border">
					<span
						class="block h-full rounded-[4px] bg-primary transition-all duration-300"
						style:width={`${angles[JOINT_ID_INDEX_MAP.SHOULDER_PAN].angle.toFixed()}%`}
					></span>
				</div>
				<p class="mt-[5px]">{angles[JOINT_ID_INDEX_MAP.SHOULDER_PAN].angle.toFixed()}&deg;</p>
			</div>
			<p class="mb-2 text-xl"><strong>Wrist Pan</strong></p>
			<Slider
				type="single"
				min={-90}
				max={90}
				bind:value={angles[JOINT_ID_INDEX_MAP.WRIST_PAN].targetAngle}
				onValueCommit={() => sendTargets(true)}
			/>
			<div class="flex flex-row gap-2">
				<div class="rounded-1/2 mb-2 mt-3 h-[8px] flex-1 rounded-[4px] border">
					<span
						class="block h-full rounded-[4px] bg-primary transition-all duration-300"
						style:width={`${angles[JOINT_ID_INDEX_MAP.WRIST_PAN].angle.toFixed()}%`}
					></span>
				</div>
				<p class="mt-[5px]">{angles[JOINT_ID_INDEX_MAP.WRIST_PAN].angle.toFixed()}&deg;</p>
			</div>
		</div>
	</div>
</div>
