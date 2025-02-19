<script lang="ts" module>
	// This is to expose the widget settings to the panel. Only put code here if you know why you are doing it!
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'TurtleSim Teleop';
	export const description = 'Demo node - control the ROS turtlesim example';
	export const group = 'ROS';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			General: {
				TurtleId: {
					type: 'number',
					description: 'The ID of the turtle to control',
					value: '1'
				}
			}
		}
	});
</script>

<script lang="ts">
	import Button from '$lib/components/ui/button/button.svelte';
	import { ArrowDown, ArrowLeft, ArrowRight, ArrowUp } from 'svelte-radix';
	import { ros } from '$lib/scripts/ros.svelte';
	import ROSLIB from 'roslib';
	import { onMount } from 'svelte';
	import { toast } from 'svelte-sonner';

	let turtleVelTopic = $derived(
		new ROSLIB.Topic({
			ros: ros.value!,
			name: `/turtle${settings.groups.General.TurtleId.value}/cmd_vel`,
			messageType: 'geometry_msgs/msg/Twist'
		})
	);

	let turtlePoseTopic = $derived(
		new ROSLIB.Topic({
			ros: ros.value!,
			name: `/turtle${settings.groups.General.TurtleId.value}/pose`,
			messageType: 'turtlesim/msg/Pose',
			throttle_rate: 500 // message rate in ms
		})
	);
	let turtlePose = $state({ x: 0, y: 0 });
	$effect(() => {
		turtleVelTopic.unsubscribe();
		turtlePoseTopic.subscribe((msg: any) => {
			// console.log(msg);
			turtlePose.x = msg.x;
			turtlePose.y = msg.y;
		});
	});

	onMount(() => {
		return () => {
			// on Unmount
			turtleVelTopic.unsubscribe();
			turtlePoseTopic.unsubscribe();
		};
	});

	const onkeydown = (e: KeyboardEvent) => {
		switch (e.key) {
			case 'ArrowUp':
			case 'w':
				turtleVelTopic.publish(new ROSLIB.Message({ linear: { x: 1 } }));
				break;
			case 'ArrowDown':
			case 's':
				turtleVelTopic.publish(new ROSLIB.Message({ linear: { x: -1 } }));
				break;
			case 'ArrowLeft':
			case 'a':
				turtleVelTopic.publish(new ROSLIB.Message({ angular: { z: 1 } }));
				break;
			case 'ArrowRight':
			case 'd':
				turtleVelTopic.publish(new ROSLIB.Message({ angular: { z: -1 } }));
				break;
		}
	};

	const spawnTurtleService = new ROSLIB.Service({
		ros: ros.value!,
		name: '/spawn',
		serviceType: 'turtlesim/srv/Spawn'
	});

	const spawnTurtle = () => {
		const serviceRequest = new ROSLIB.ServiceRequest({
			x: 5,
			y: 5,
			theta: 0
		});

		spawnTurtleService.callService(serviceRequest, (response) => {
			toast.info(`Spawned turtle with id: ${response.name}`);
		});
	};

	const killTurtleService = new ROSLIB.Service({
		ros: ros.value!,
		name: '/kill',
		serviceType: 'turtlesim/srv/Kill'
	});

	const killTurtle = () => {
		const serviceRequest = new ROSLIB.ServiceRequest({
			name: `turtle${settings.groups.General.TurtleId.value}`
		});

		killTurtleService.callService(serviceRequest, () => {
			settings.groups.General.TurtleId.value = undefined;
		});
	};

	const resetTurtleService = new ROSLIB.Service({
		ros: ros.value!,
		name: '/reset',
		serviceType: 'std_srvs/srv/Empty'
	});

	const resetTurtle = () => {
		resetTurtleService.callService(new ROSLIB.ServiceRequest({}), () => {});
	};
</script>

<div class="flex-col justify-between">
	<div class="flex-col">
		<div class="flex">
			<div class="my-auto w-fit min-w-[140px] flex-col space-y-2">
				<div class="flex">
					<div class="mx-auto">
						<Button
							size="sm"
							onclick={() => turtleVelTopic.publish(new ROSLIB.Message({ linear: { x: 1 } }))}
						>
							<ArrowUp />
						</Button>
					</div>
				</div>
				<div class="flex">
					<div class="mx-auto space-x-1">
						<Button
							size="sm"
							onclick={() => turtleVelTopic.publish(new ROSLIB.Message({ angular: { z: 1 } }))}
						>
							<ArrowLeft />
						</Button>
						<Button
							size="sm"
							onclick={() => turtleVelTopic.publish(new ROSLIB.Message({ linear: { x: -1 } }))}
							><ArrowDown /></Button
						>
						<Button
							size="sm"
							onclick={() => turtleVelTopic.publish(new ROSLIB.Message({ angular: { z: -1 } }))}
							><ArrowRight /></Button
						>
					</div>
				</div>
			</div>
			<div class="ml-4 flex-col space-y-2">
				<Button variant="outline" onclick={spawnTurtle}>New turtle</Button>
				<Button variant="outline" onclick={killTurtle}>Kill active turtle</Button>
				<Button variant="outline" onclick={resetTurtle}>Reset sim</Button>
			</div>
		</div>
	</div>
	<div class="mt-auto">
		<p class="mt-2 opacity-60">
			Position: ({Math.round(turtlePose.x)}, {Math.round(turtlePose.y)})
		</p>
		<p class="opacity-60">Active turtle id: {settings.groups.General.TurtleId.value}</p>
	</div>
</div>

<svelte:window {onkeydown} />
