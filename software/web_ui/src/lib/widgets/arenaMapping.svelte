<script lang="ts" module>
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	// Internal identifiers follow: American spelling + snake_case + minimal abbreviations.
	const widget_name = 'Arena Map Editor';
	const widget_description = 'Create the YAML file that Perseus will use to navigate';
	const widget_group = 'ROS';
	const is_ros_dependent = true;

	const widget_settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
<<<<<<< HEAD
=======

>>>>>>> e2ab3e4e (fix(map-gui): simplified arena mapping publisher and subscriber)
	export {
		widget_name as name,
		widget_description as description,
		widget_group as group,
		is_ros_dependent as isRosDependant,
		widget_settings as settings
	};


	// function nextWaypointName(): string {
	// 	throw new Error('Function not implemented.');
	// }
</script>

<script lang="ts">
	import * as ROSLIB from 'roslib';
	import { getRosConnection as getRosConnection } from '$lib/scripts/rosBridge.svelte';
	//import Button from "$lib/components/ui/button/button.svelte";
	import { ScrollArea } from '$lib/components/ui/scroll-area/index';
	import { Square } from 'svelte-radix';
<<<<<<< HEAD
	import type { Point } from 'chart.js';
=======
>>>>>>> e2ab3e4e (fix(map-gui): simplified arena mapping publisher and subscriber)
	//import { response } from 'express';

	type Mode = 'waypoint' | 'border' | 'origin'| 'manual';
	type Direction = 'up' | 'down' | 'left' | 'right' | 'unselected';
	type rosStringMessage = { data: string };

	type WaypointRow = {
		id: string;
		name: string;
		hexadecimalColor: string;
		clickX: number;
		clickY: number;
		centroidX: number;
		centroidY: number;
		relativeX: number;
		relativeY: number;
		yaw: number;
		contour: number[][];
	};

	type OriginRow = {
		id: string;
		name: string;
		hexadecimalColor: string;
		clickX: number;
		clickY: number;
		centroidX: number;
		centroidY: number;
	};

	type ArrowRow = {
		id: string;
		name: string;
		hexadecimalColor: string;
		clickX: number;
		clickY: number;
		centroidX: number;
		centroidY: number;
	};
	type WaypointPoint = {
		xCoordinate: number;
		yCoordinate: number;
	}

	let imageElement: HTMLImageElement | null = null;

<<<<<<< HEAD
	const mapImageId = 'Cropped_ARCH_2025_Autonomous_map_12.png';
	// mapImageUrl = `http://localhost:8000/${mapImageId}`;
=======
	const mapImageId = 'Cropped_ARCH_2025_Autonomous_map_2.png';
	const mapImageUrl = `http://localhost:8000/${mapImageId}`;
>>>>>>> e2ab3e4e (fix(map-gui): simplified arena mapping publisher and subscriber)

	//Ros topics to listen to requests and reply
	const requestTopicName = '/map_editor/request';
	const responseTopicName = '/map_editor/response';

	// Setting the hue saturation and value thresholds for the image extraction - can be changed to adjust extraction results
	const defaultHueTolerance = 10;
	const defaultSaturationTolerance = 60;
	const defaultValueTolerance = 60;

	let statusMessage = $state('Ready');
	let mode = $state<Mode>('waypoint');

	// last response visuals
	let contour = $state<number[][]>([]);
	let currentClickPosition = $state<{ x: number; y: number } | null>(null);
	let centroid = $state<[number, number] | null>(null);
	let sampleImageHexadecimalColor = $state<string>('');
	let svgViewBox = $state('0 0 1 1');

	// stored waypoints
	let waypoints = $state<WaypointRow[]>([]);
	let waypointToggle = $state(0);
<<<<<<< HEAD

=======
>>>>>>> e2ab3e4e (fix(map-gui): simplified arena mapping publisher and subscriber)
	// 0 -> selecting waypoint
	// 1 -> selecting arrow
	let lastID = $state('');

	//stored origin
	let origins = $state<OriginRow[]>([]);

	//stored arrow
	let arrows = $state<ArrowRow[]>([]);

	let borderContour = $state<number[][]>([]);
<<<<<<< HEAD

	let pendingSaveId: string | null = null; 
=======
>>>>>>> e2ab3e4e (fix(map-gui): simplified arena mapping publisher and subscriber)

	// ROS request/response plumbing
	let requestTopic: ROSLIB.Topic<rosStringMessage> | null = null;
	let responseTopic: ROSLIB.Topic<rosStringMessage> | null = null;

	//scale
	let scale = $state(1);
	let mapHeight = $state(0);

	//origin
	let positiveXDirection = $state<Direction>('up');
	let positiveYDirection = $state<Direction>('left');
	let xOriginAverage = $state(0);
	let yOriginAverage = $state(0);
	// bind the next arrow click to the waypoint that was just added
	let pendingYawWaypointId: string | null = null;

<<<<<<< HEAD
<<<<<<< HEAD
	$effect(() => {
		const rosConnection = getRosConnection();
		if (rosConnection) {
			requestTopic = new ROSLIB.Topic({
				ros: rosConnection,
				name: requestTopicName,
				messageType: 'std_msgs/msg/String'
=======
	function calculate_origin() {
		let total_x = 0;
		let total_y = 0;
		origins.forEach((origin, o) => {
			total_x += origin.centroid_x;
			total_y += origin.centroid_y;
=======
	$effect(() => {
		const ros_connection = getRosConnection();
		if (ros_connection) {
			requestTopic = new ROSLIB.Topic({
				ros: ros_connection,
				name: requestTopicName,
				messageType: 'std_msgs/msg/String'
			});
			responseTopic = new ROSLIB.Topic({
				ros: ros_connection,
				name: responseTopicName,
				messageType: 'std_msgs/msg/String'
			});
			responseTopic.subscribe(onResponseMessage);
		}
		else {
			responseTopic?.unsubscribe();
			requestTopic = null;
			responseTopic = null;

			// pendingRequests.forEach((pending_entry) => clearTimeout(pending_entry.timeoutHandle));
			// pendingRequests.clear();
		}
	});

	const onResponseMessage = (message: rosStringMessage) => {
		let response: any;
		response = JSON.parse(message.data);
		if (mode === 'border') {
			borderContour = Array.isArray(response.contour) ? response.contour : [];
		}

		sampleImageHexadecimalColor = String(response.sample_image_hex ?? '');
		centroid = Array.isArray(response.centroid) ? (response.centroid as [number, number]) : null;
		contour = Array.isArray(response.contour) ? response.contour : [];

		if (mode === 'waypoint') {
			if (waypointToggle == 0) {
				add_waypoint_from_response(response, contour);
				statusMessage = centroid
					? `Waypoint added — centroid (${centroid[0]}, ${centroid[1]}) ${sampleImageHexadecimalColor}`
					: `Waypoint added ${sampleImageHexadecimalColor}`;
				waypointToggle = 1;
			}
			else {
				addAngleFromResponse(response);
				statusMessage = centroid
					? `Arrow added — centroid (${centroid[0]}, ${centroid[1]}) ${sampleImageHexadecimalColor}`
					: `Arrow added ${sampleImageHexadecimalColor}`;
				waypointToggle = 0;
				calculate_angle();
			}
		} else if (mode === 'origin') {
			add_origin_from_response(response);
			statusMessage = centroid
				? `Origin added — centroid (${centroid[0]}, ${centroid[1]}) ${sampleImageHexadecimalColor}`
				: `Origin added ${sampleImageHexadecimalColor}`;
		} else {
			statusMessage = `Border OK ${sampleImageHexadecimalColor}`;
		}
	}


	function calculate_origin() {
		let total_x = 0;
		let total_y = 0;
		origins.forEach((origin) => {
			total_x += origin.centroidX;
			total_y += origin.centroidY;
>>>>>>> e2ab3e4e (fix(map-gui): simplified arena mapping publisher and subscriber)
		});
		x_origin = total_x / origins.length;
		y_origin = total_y / origins.length;

		waypoints.forEach((waypoint) => {
			switch(positive_x_direction) {
				case "left":
					waypoint.relative_x = x_origin - waypoint.centroid_x;
					break;
				case "right":
					waypoint.relative_x = waypoint.centroid_x - x_origin;
					break;
				case "down":
					waypoint.relative_x = waypoint.centroid_y - y_origin;
					break;
				case "up":
					waypoint.relative_x = y_origin - waypoint.centroid_y;
			}
			switch(positive_y_direction) {
				case "left":
					waypoint.relative_y = x_origin - waypoint.centroid_x;
					break;
				case "right":
					waypoint.relative_y = waypoint.centroid_x - x_origin;
					break;
				case "down":
					waypoint.relative_y = waypoint.centroid_y - y_origin;
					break;
				case "up":
					waypoint.relative_y = y_origin - waypoint.centroid_y;
			}
		});
	}

	function calculate_angle() {
		waypoints.forEach((waypoint, w) => {
			const delta_x = waypoint.centroidX - arrows[w].centroidX;
			const delta_y = waypoint.centroidY - arrows[w].centroidY;

<<<<<<< HEAD
					const delta_x = waypoint.centroid_x - arrow.centroid_x;
					const delta_y = waypoint.centroid_y - arrow.centroid_y;
					
					console.log(delta_x);
					console.log(delta_y);
					console.log(waypoint.yaw);
				
					if (delta_x == 0) {
						if (waypoint.centroid_y <= arrow.centroid_y) {
							waypoint.yaw = Math.PI;
						}
						else if(waypoint.centroid_y >= arrow.centroid_y){
							waypoint.yaw = 0;
						}
					}
					else if (delta_y == 0) {
						if (waypoint.centroid_x >= arrow.centroid_x){
							waypoint.yaw = Math.PI/2;
						}
						else if(waypoint.centroid_x <= arrow.centroid_x){
							waypoint.yaw = 3*Math.PI/2;
						}
					}
					else {
						if ((delta_x > 0) && (delta_y > 0)) {
							waypoint.yaw = Math.atan(delta_x/delta_y);
						}
						else if((delta_x < 0) && (delta_y > 0)) {
							waypoint.yaw = 3*Math.PI/2 + Math.abs(Math.atan(delta_y/delta_x));
						}
						else if((delta_x < 0) && (delta_y < 0)){
							waypoint.yaw = Math.PI + Math.abs(Math.atan(delta_x/delta_y));
						}
						else if((delta_x > 0) && (delta_y < 0)) {
							waypoint.yaw = Math.PI/2 + Math.abs(Math.atan(delta_x/delta_y));
						}
					}
=======
			if (delta_x == 0) {
				if (waypoint.centroidY <= arrows[w].centroidY) {
					waypoint.yaw = Math.PI;
>>>>>>> e2ab3e4e (fix(map-gui): simplified arena mapping publisher and subscriber)
				}
				else if(waypoint.centroidY >= arrows[w].centroidY){
					waypoint.yaw = 0;
				}
			}
			else if (delta_y == 0) {
				if (waypoint.centroidX >= arrows[w].centroidX){
					waypoint.yaw = Math.PI/2;
				}
				else if(waypoint.centroidX <= arrows[w].centroidX){
					waypoint.yaw = 3*Math.PI/2;
				}
			}
			else {
				if ((delta_x > 0) && (delta_y > 0)) {
					waypoint.yaw = Math.atan(delta_x/delta_y);
				}
				else if((delta_x < 0) && (delta_y > 0)) {
					waypoint.yaw = 3*Math.PI/2 + Math.abs(Math.atan(delta_y/delta_x));
				}
				else if((delta_x < 0) && (delta_y < 0)){
					waypoint.yaw = Math.PI + Math.abs(Math.atan(delta_x/delta_y));
				}
				else if((delta_x > 0) && (delta_y < 0)) {
					waypoint.yaw = Math.PI/2 + Math.abs(Math.atan(delta_x/delta_y));
				}
			}
		});
	}

<<<<<<< HEAD
	function to_waypoint() {
		waypoint_toggle ? status_message = 'Click on waypoint' : status_message = 'Click on arrow';
	}

	const pendingRequests = new Map<
		string,
		{ resolve: (response: any) => void; timeoutHandle: any }
	>();

	function generate_id() {
		last_id = typeof crypto !== 'undefined' && 'randomUUID' in crypto
=======
	function generateID() {
		lastID = typeof crypto !== 'undefined' && 'randomUUID' in crypto
>>>>>>> e2ab3e4e (fix(map-gui): simplified arena mapping publisher and subscriber)
			? crypto.randomUUID()
			: `${Date.now()}-${Math.random().toString(16).slice(2)}`;
		return lastID;
	}

	function clamp(value: number, min_value: number, max_value: number) {
		return Math.max(min_value, Math.min(max_value, value));
	}

	//Converts the click on the image to the pixle coordinates on the image
	function clickToNaturalPosition(event: MouseEvent) {
		if (!imageElement) return null;

		const image_rectangle = imageElement.getBoundingClientRect();
		const displayX = event.clientX - image_rectangle.left;
		const displayY = event.clientY - image_rectangle.top;

		const naturalX = Math.round(displayX * (imageElement.naturalWidth / image_rectangle.width));
		const naturalY = Math.round(displayY * (imageElement.naturalHeight / image_rectangle.height));

		return {
			x: clamp(naturalX, 0, imageElement.naturalWidth - 1),
			y: clamp(naturalY, 0, imageElement.naturalHeight - 1)
		};
	}

<<<<<<< HEAD
		const id = generateId();
		return new Promise((resolve, reject) => {
			const timeoutHandle = setTimeout(() => {
				pendingRequests.delete(id);
				reject(new Error(`Timeout waiting for response (${id})`));
			}, timeoutMilliseconds);

			pendingRequests.set(id, { resolve, timeoutHandle });

			requestTopic!.publish({
				data: JSON.stringify({ id, ...payload })
>>>>>>> e67ad1cf (chore: Format and lint)
			});
			responseTopic = new ROSLIB.Topic({
				ros: rosConnection,
				name: responseTopicName,
				messageType: 'std_msgs/msg/String'
			});
			responseTopic.subscribe(onResponseMessage);
		}
		else {
			responseTopic?.unsubscribe();
			requestTopic = null;
			responseTopic = null;

			// pendingRequests.forEach((pending_entry) => clearTimeout(pending_entry.timeoutHandle));
			// pendingRequests.clear();
		}
	});

	//direction mapping from image coordinates (dx right+, dy down+)
	function axisFromImage(dir: Direction, dx: number, dy: number): number {
		switch (dir) {
			case 'right': return dx;
			case 'left':  return -dx;
			case 'down':  return dy;
			case 'up':    return -dy;
			default:      return 0;
		}
=======
	// creates the waypoint names
	function nextWaypointName() {
		const waypointNumber = waypoints.length + 1;
		return `WP${String(waypointNumber)}`;
>>>>>>> e2ab3e4e (fix(map-gui): simplified arena mapping publisher and subscriber)
	}

	const onResponseMessage = (message: rosStringMessage) => {
		const response: any = JSON.parse(message.data);
		if (pendingSaveId && response?.id === pendingSaveId) {
			pendingSaveId = null;

			statusMessage = response?.ok
			? `Saved: ${response.saved_path ?? 'OK'}`
			: `Save failed: ${response?.message ?? 'Unknown error'}`;

			return;
		}

		const hexadecimalColor = getSampleHex(response);
		const waypointContour = extractedContour;


		waypoints = [
			...waypoints,
			{
				id: generate_id(),
				name: next_waypoint_name(),
				hexadecimal_color,
				click_x,
				click_y,
				centroid_x,
				centroid_y,
				yaw:0
			}
		];
	}

	function add_angle_from_response(response: any) {
		if (!response?.ok) return;
		if (!Array.isArray(response.centroid) || response.centroid.length !== 2) return;

		const centroid_x = Number(response.centroid[0]);
		const centroid_y = Number(response.centroid[1]);

		// Prefer server echo; fall back to local current_click_position.
		const click_x = Number(response.sample_x ?? current_click_position?.x);
		const click_y = Number(response.sample_y ?? current_click_position?.y);

		if (
			!Number.isFinite(click_x) ||
			!Number.isFinite(click_y) ||
			!Number.isFinite(centroid_x) ||
			!Number.isFinite(centroid_y)
		) {
			return;
		}
		// extraction replies
		if (mode === 'manual') return;

		if (mode === 'border') {
			borderContour = Array.isArray(response.contour) ? response.contour : [];
		}

		sampleImageHexadecimalColor = String(response.sampleImageHexValue ?? '');
		centroid = Array.isArray(response.centroid) ? (response.centroid as [number, number]) : null;
		contour = Array.isArray(response.contour) ? response.contour : [];


		if (mode === 'waypoint') {
			if (waypointToggle === 0) {
				addWaypointFromResponse(response, contour);

				statusMessage = centroid
					? `Waypoint added — centroid (${centroid[0]}, ${centroid[1]}) ${sampleImageHexadecimalColor}`
					: `Waypoint added ${sampleImageHexadecimalColor}`;

				waypointToggle = 1;
			} else {
				const arrow = addAngleFromResponse(response);
				statusMessage = centroid
					? `Arrow added — centroid (${centroid[0]}, ${centroid[1]}) ${sampleImageHexadecimalColor}`
					: `Arrow added ${sampleImageHexadecimalColor}`;

				waypointToggle = 0;

				if (pendingYawWaypointId && arrow) {
					applyArrowYawToWaypoint(pendingYawWaypointId, arrow);
					pendingYawWaypointId = null;
				}
			}
	}
}


	function add_origin_from_response(response: any) {
		if (!response?.ok) return;
		if (!Array.isArray(response.centroid) || response.centroid.length !== 2) return;

		const centroid_x = Number(response.centroid[0]);
		const centroid_y = Number(response.centroid[1]);

		// Prefer server echo; fall back to local current_click_position.
		const click_x = Number(response.sample_x ?? current_click_position?.x);
		const click_y = Number(response.sample_y ?? current_click_position?.y);

		if (
			!Number.isFinite(click_x) ||
			!Number.isFinite(click_y) ||
			!Number.isFinite(centroid_x) ||
			!Number.isFinite(centroid_y)
		) {
			return;
		}

		const hexadecimal_color = String(response.sample_image_hex ?? '');

		origins = [
			...origins,
			{
				id: generate_id(),
				name: next_origin_name(),
				hexadecimal_color,
				click_x,
				click_y,
				centroid_x,
				centroid_y
			}
		];
	}

	function deleteWaypoints(id: string) {
		waypoints = waypoints.filter((waypoint) => waypoint.id !== id);
	}

	function delete_origin(id: string) {
		origins = origins.filter((origin) => origin.id !== id);
	}

	function clear_waypoints() {
		waypoints = [];
	}
	function updateYaw(id: string, yawDeg: number) {
		const yaw = Number.isFinite(yawDeg) ? yawDeg : 0;
		waypoints = waypoints.map((w) => (w.id === id ? { ...w, yaw } : w));
	}

	function updateName(id: string, name: string) {
		waypoints = waypoints.map((waypoint) => (waypoint.id === id ? { ...waypoint, name } : waypoint));
	}

<<<<<<< HEAD
	function buildYaml(): string {
		const lines: string[] = [];
		
		lines.push('waypoints:');

		for (const waypoint of waypoints) {
			const yawRadians = ((waypoint.yaw)*Math.PI)/180;
			lines.push(`- name: ${waypoint.name}`);
			lines.push(`x: ${waypoint.relative_x / scale}`);
			lines.push(`y: ${waypoint.relative_y / scale}`);
			lines.push(`yaw: ${waypoint.yaw}`)
		}
=======
	function wrapPi(rad: number) {
		return Math.atan2(Math.sin(rad), Math.cos(rad));
	}

	function buildYaml(): string {
		const lines: string[] = [];
		lines.push('waypoints:');

			for (const waypoint of waypoints) {
				const yawRadians = ((waypoint.yaw)*Math.PI)/180;
				lines.push(`- name: ${waypoint.name}`);
				lines.push(`x: ${waypoint.relativeX / scale}`);
				lines.push(`y: ${waypoint.relativeY / scale}`);
				lines.push(`yaw: ${waypoint.yaw}`)
			}
>>>>>>> e2ab3e4e (fix(map-gui): simplified arena mapping publisher and subscriber)

		return lines.join('\n');
	}
	  
	function drawRotatedRectangle(waypoint:waypointRow){
			const arrowLength = 25; 

<<<<<<< HEAD
			const yawDegrees = ((waypoint.yaw-90) * Math.PI) / 180;

			const yawX1Coordinate = waypoint.centroidX;
			const yawY1Coordinate = waypoint.centroidY; 

			const yawX2Coordinate = yawX1Coordinate + Math.cos(yawDegrees) * arrowLength;
			const yawY2Coordinate = yawY1Coordinate + Math.sin(yawDegrees) * arrowLength;
			return{yawX1Coordinate,yawX2Coordinate, yawY1Coordinate, yawY2Coordinate};
=======
	function drawRotatedRectangle(waypoint: WaypointRow) {
		const arrowLength = 25;

		// Same convention as YAML: N=0, E=-pi/2, W=+pi/2
		const t = (-waypoint.yaw * Math.PI) / 180;

		const yawX1Coordinate = waypoint.centroidX;
		const yawY1Coordinate = waypoint.centroidY;

		// t=0 points up: dx=0, dy=-1
		const yawX2Coordinate = yawX1Coordinate + Math.sin(t) * arrowLength;
		const yawY2Coordinate = yawY1Coordinate - Math.cos(t) * arrowLength;

		return { yawX1Coordinate, yawX2Coordinate, yawY1Coordinate, yawY2Coordinate };
>>>>>>> e2ab3e4e (fix(map-gui): simplified arena mapping publisher and subscriber)
	}

// Function to help translate the difference in case variables from the back end to front end
	function getSampleHex(response: any): string {
		return String(
<<<<<<< HEAD
			response?.sample_image_hex ??       
			response?.sampleImageHex ??      
			response?.sample_image_hexadecimal_color ?? 
=======
			response?.sample_image_hex ??
			response?.sampleImageHex ??
			response?.sampleImageHexadecimalColor ??
>>>>>>> e2ab3e4e (fix(map-gui): simplified arena mapping publisher and subscriber)
			''
		);
	}

	async function saveYamlToScripts() {
<<<<<<< HEAD
		const yaml_text = buildYaml();
		try {
			statusMessage = 'Uploading YAML file to Perseus';
			const response = await sendRequest({
				op: 'save_yaml',
				file_name: 'Waypoints.yaml',
				yaml_text
			});
			if (!response?.ok){
				statusMessage = response?.message ?? 'Save failed';
				return;
			}
    		statusMessage = `Saved: ${response.saved_path ?? 'OK'}`;
		} catch (e:any) {
			statusMessage = `Save error: ${e?.message ?? String(e)}`;
		}
		 if (mode === 'origin') {
			addOriginFromResponse(response);
			// calculateOrigin(); 
			statusMessage = centroid
				? `Origin added — centroid (${centroid[0]}, ${centroid[1]}) ${sampleImageHexadecimalColor}`
				: `Origin added ${sampleImageHexadecimalColor}`;
		} 
		// else {
		// 	statusMessage = `Border OK ${sampleImageHexadecimalColor}`;
		// }
	}

	function toRelative(centroidX: number, centroidY: number) {
		if (origins.length === 0) {
			return { relativeX: centroidX, relativeY: centroidY };
=======
	// 	const yaml_text = buildYaml();
	// 	try {
	// 		statusMessage = 'Uploading YAML file to Perseus';
	// 		const response = await sendRequest({
	// 			op: 'save_yaml',
	// 			file_name: 'Waypoints.yaml',
	// 			yaml_text
	// 		});
	// 		if (!response?.ok){
	// 			statusMessage = response?.message ?? 'Save failed';
	// 			return;
	// 		}
    // 		statusMessage = `Saved: ${response.saved_path ?? 'OK'}`;
	// 	} catch (e:any) {
	// 		statusMessage = `Save error: ${e?.message ?? String(e)}`;
	// 	}
	}

	async function onMapClick(event: MouseEvent) {
		let x_dir;
		let y_dir;
		if (document.getElementById("p_direction_x")) {
			const x_select = document.getElementById("p_direction_x") as HTMLSelectElement;
			const y_select = document.getElementById("p_direction_y") as HTMLSelectElement;
			x_dir = x_select.value;
			y_dir = y_select.value;
		}
		else {
			x_dir = positive_x_direction;
			y_dir = positive_y_direction;
>>>>>>> e2ab3e4e (fix(map-gui): simplified arena mapping publisher and subscriber)
		}

		// Default to common map convention if user hasn't selected directions yet
		const xDir: Direction = positiveXDirection === 'unselected' ? 'left' : positiveXDirection;
		const yDir: Direction = positiveYDirection === 'unselected' ? 'up' : positiveYDirection;

		const dx = centroidX - xOriginAverage; // +right
		const dy = centroidY - yOriginAverage; // +down

		return {
			relativeX: axisFromImage(xDir, dx, dy),
			relativeY: axisFromImage(yDir, dx, dy)
		};
	}

	function calculateOrigin() {
		if (origins.length === 0) return;

		let totalSumXValuesOrigin = 0;
		let totalSumYValuesOrigin = 0;

		origins.forEach((o) => {
			totalSumXValuesOrigin += o.centroidX;
			totalSumYValuesOrigin += o.centroidY;
		});

		xOriginAverage = totalSumXValuesOrigin / origins.length;
		yOriginAverage = totalSumYValuesOrigin / origins.length;

		// recompute relative coords for all waypoints using updated origin + directions
		waypoints = waypoints.map((waypoint) => {
			const rel = toRelative(waypoint.centroidX, waypoint.centroidY);
			return { ...waypoint, relativeX: rel.relativeX, relativeY: rel.relativeY };
		});
	}
	
	function calculateAngle() {
		const n = Math.min(waypoints.length, arrows.length);
		waypoints = waypoints.map((waypoint, i) => {
			if (i >= n) return waypoint;

			const arrow = arrows[i];
			const dx = arrow.centroidX - waypoint.centroidX;
			const dy = arrow.centroidY - waypoint.centroidY;
			const angleOfArrowDegrees = ((Math.atan2(-dx, -dy) * 180) / Math.PI + 360) % 360;
			return { ...waypoint, yaw: angleOfArrowDegrees };
		});

	}

	function generateID() {
		lastID = typeof crypto !== 'undefined' && 'randomUUID' in crypto
			? crypto.randomUUID()
			: `${Date.now()}-${Math.random().toString(16).slice(2)}`;
		return lastID;
	}

	function clamp(value: number, minimumImageHeight: number, maximumImageHeight: number) {
		return Math.max(minimumImageHeight, Math.min(maximumImageHeight, value));
	}

	//Converts the click on the image to the pixle coordinates on the image
	function clickToNaturalPosition(event: MouseEvent) {
		if (!imageElement) return null;

		const imageRectangle = imageElement.getBoundingClientRect();
		const displayX = event.clientX - imageRectangle.left;
		const displayY = event.clientY - imageRectangle.top;

		const naturalX = Math.round(displayX * (imageElement.naturalWidth / imageRectangle.width));
		const naturalY = Math.round(displayY * (imageElement.naturalHeight / imageRectangle.height));

		return {
			x: clamp(naturalX, 0, imageElement.naturalWidth - 1),
			y: clamp(naturalY, 0, imageElement.naturalHeight - 1)
		};
	}

	// creates the waypoint names
	function nextWaypointName() {
		const waypointNumber = waypoints.length + 1;
		return `WP${String(waypointNumber)}`;
	}

	function nextOriginName() {
		const originNumber = origins.length + 1;
		return `O${String(originNumber)}`;
	}

	function nextArrowName() {
		const arrowNumber = arrows.length + 1;
		return `AR${String(arrowNumber)}`;
	}

	function addManualWaypoint(clickPosition: { x: number; y: number }) {
		const clickX = clickPosition.x;
		const clickY = clickPosition.y;

		const id = generateID();
		const rel = toRelative(clickX, clickY);

		waypoints = [
			...waypoints,
			{
				id,
				name: nextWaypointName(),
				hexadecimalColor: '#ffffff',
				clickX,
				clickY,
				centroidX: clickX,
				centroidY: clickY,
				relativeX: rel.relativeX,
				relativeY: rel.relativeY,
				yaw: 0,
				contour: []
			}
		];

		pendingYawWaypointId = id;
		statusMessage = `Manual waypoint added @ (${clickX}, ${clickY})`;
	}

	function addManualOrigin(clickPosition: { x: number; y: number }) {
		const clickX = clickPosition.x;
		const clickY = clickPosition.y;

		origins = [
			...origins,
			{
				id: generateID(),
				name: nextOriginName(),        
				hexadecimalColor: '#ffffff',      
				clickX,
				clickY,
				centroidX: clickX,
				centroidY: clickY
			}
		];

		statusMessage = `Origin point added @ (${clickX}, ${clickY})`;
	}



	function addWaypointFromResponse(response: any, extractedContour: number[][]) {
		if (!response?.ok) return;
		if (!Array.isArray(response.centroid) || response.centroid.length !== 2) return;

		const centroidX = Number(response.centroid[0]);
		const centroidY = Number(response.centroid[1]);

		const clickX = Number(response.sampleXPosition ?? currentClickPosition?.x);
		const clickY = Number(response.sampleYPosition ?? currentClickPosition?.y);

		if (!Number.isFinite(clickX) || !Number.isFinite(clickY) || !Number.isFinite(centroidX) || !Number.isFinite(centroidY)) {
			return;
		}

		const id = generateID();
		const rel = toRelative(centroidX, centroidY);

		waypoints = [
			...waypoints,
			{
				id,
				name: nextWaypointName(),
				hexadecimalColor: getSampleHex(response),
				clickX,
				clickY,
				centroidX,
				centroidY,
				relativeX: rel.relativeX,
				relativeY: rel.relativeY,
				yaw: 0,
				contour: extractedContour
			}
		];

		pendingYawWaypointId = id;
	}

	function addAngleFromResponse(response: any): ArrowRow | null {
		if (!response?.ok) return null;
		if (!Array.isArray(response.centroid) || response.centroid.length !== 2) return null;

		const centroidX = Number(response.centroid[0]);
		const centroidY = Number(response.centroid[1]);

		const clickX = Number(response.sampleXPosition ?? currentClickPosition?.x);
		const clickY = Number(response.sampleYPosition ?? currentClickPosition?.y);
		if (!Number.isFinite(clickX) || !Number.isFinite(clickY) || !Number.isFinite(centroidX) || !Number.isFinite(centroidY)) return null;

		const arrow: ArrowRow = {
			id: generateID(),
			name: nextArrowName(),
			hexadecimalColor: String(response.sampleImageHexValue ?? ''),
			clickX,
			clickY,
			centroidX,
			centroidY
		};

		arrows = [...arrows, arrow];
		return arrow;
	}

	function applyArrowYawToWaypoint(waypointId: string, arrow: ArrowRow) {
		const wp = waypoints.find((w) => w.id === waypointId);
		if (!wp) return;

		const dx = arrow.centroidX - wp.centroidX; // +right
		const dy = arrow.centroidY - wp.centroidY; // +down

		// 0=N, 90=W, 180=S, 270=E
		const yawDeg = ((Math.atan2(-dx, -dy) * 180) / Math.PI + 360) % 360;

		waypoints = waypoints.map((w) => (w.id === waypointId ? { ...w, yaw: yawDeg } : w));
	}
	function addOriginFromResponse(response: any) {
		if (!response?.ok) return;
		if (!Array.isArray(response.centroid) || response.centroid.length !== 2) return;

		const centroidX = Number(response.centroid[0]);
		const centroidY = Number(response.centroid[1]);

		// Prefer server echo; fall back to local currentClickPosition.
		const clickX = Number(response.sampleXPosition ?? currentClickPosition?.x);
		const clickY = Number(response.sampleYPosition ?? currentClickPosition?.y);

		if (
			!Number.isFinite(clickX) ||
			!Number.isFinite(clickY) ||
			!Number.isFinite(centroidX) ||
			!Number.isFinite(centroidY)
		) {
			return;
		}

		const hexadecimalColor = String(response.sampleImageHexValue ?? '');

		origins = [
			...origins,
			{
				id: generateID(),
				name: nextOriginName(),
				hexadecimalColor,
				clickX,
				clickY,
				centroidX,
				centroidY
			}
		];
	}

	function deleteWaypoint(id: string) {
		waypoints = waypoints.filter((waypoint) => waypoint.id !== id);
	}

	function deleteOrigin(id: string) {
		origins = origins.filter((origin) => origin.id !== id);
	}

	function clearWaypoints() {
		waypoints = [];
	}
	function updateYaw(id: string, yawDeg: number) {
		let yaw = Number.isFinite(yawDeg) ? yawDeg : 0;
		yaw = ((yaw % 360) + 360) % 360; // keep within 0..359.999
		waypoints = waypoints.map((w) => (w.id === id ? { ...w, yaw } : w));
	}


	function updateName(id: string, name: string) {
		waypoints = waypoints.map((waypoint) => (waypoint.id === id ? { ...waypoint, name } : waypoint));
	}

	function ChangeInPointDistance(Point1Position: WaypointPoint, Point2Position: WaypointPoint) {
		const DistanceBetweenPointsX = Point1Position.xCoordinate - Point2Position.xCoordinate;
		const DistanceBetweenPointsY = Point1Position.yCoordinate - Point2Position.yCoordinate;
		return DistanceBetweenPointsX * DistanceBetweenPointsX + DistanceBetweenPointsY * DistanceBetweenPointsY;
	}

	function waypointMeters(waypoint: WaypointRow): WaypointPoint {
		const pixelsPerMeter = Number.isFinite(scale) && scale > 0 ? scale : 1;
		return {
			xCoordinate: waypoint.relativeX / pixelsPerMeter,
			yCoordinate: waypoint.relativeY / pixelsPerMeter
		};
	}

	function orderWaypointsByPath(input: WaypointRow[]): WaypointRow[] {
		const remaining = [...input];
		const ordered: WaypointRow[] = [];

		let current: WaypointPoint = { xCoordinate: 0, yCoordinate: 0 };

		while (remaining.length > 0) {
			let bestIndex = 0;
			let bestDistance = Infinity;

			for (let i = 0; i < remaining.length; i++) {
				const wpPoint = waypointMeters(remaining[i]);
				const distance = ChangeInPointDistance(current, wpPoint);

				if (
					distance < bestDistance ||
					(distance === bestDistance && remaining[i].name < remaining[bestIndex].name)
				) {
					bestDistance = distance;
					bestIndex = i;
				}
			}

			const nextWaypoint = remaining.splice(bestIndex, 1)[0];
			ordered.push(nextWaypoint);
			current = waypointMeters(nextWaypoint);
		}

		return ordered;
	}
	function wrapPi(rad: number) {
		return Math.atan2(Math.sin(rad), Math.cos(rad));
	}
	function roundJsonOutput(n: number, dp: number) {
		return Number.isFinite(n) ? Number(n.toFixed(dp)) : 0;
	}
	function buildJson(): string {
			const ordered = orderWaypointsByPath(waypoints);

			const payload = {
			waypoints: ordered.map((w) => {
				const xRelativeCoordinate = w.relativeX / scale;
				const yRelativeCoordinate = w.relativeY / scale;

				// yaw stored as DEGREES in UI but output yaw in RADIANS for navigation
				const YawOutputRadians = wrapPi((w.yaw * Math.PI) / 180);

				return {
					name: w.name,
					x: roundJsonOutput(xRelativeCoordinate, 3),
					y: roundJsonOutput(yRelativeCoordinate, 3),
					yaw: roundJsonOutput(YawOutputRadians, 6)
				};
			})
	};

	return JSON.stringify(payload, null, 2);
	}

	function drawRotatedRectangle(waypoint: WaypointRow) {
		const arrowLength = 25;

		// Same convention as YAML: N=0, E=-pi/2, W=+pi/2
		const t = (-waypoint.yaw * Math.PI) / 180;

		const yawX1Coordinate = waypoint.centroidX;
		const yawY1Coordinate = waypoint.centroidY;
		const yawX2Coordinate = yawX1Coordinate + Math.sin(t) * arrowLength;
		const yawY2Coordinate = yawY1Coordinate - Math.cos(t) * arrowLength;

		return { yawX1Coordinate, yawX2Coordinate, yawY1Coordinate, yawY2Coordinate };
	}

	// Function to help translate the difference in case variables from the back end to front end
	function getSampleHex(response: any): string {
		return String(
			response?.sampleImageHexValue ??
			response?.sampleImageHex ??
			response?.sampleImageHexadecimalColor ??
			''
		);
	}
	// Function that sets the file name of the saved file created in buildJson()
	function saveJsonToScripts() {
		if (!requestTopic) {
			statusMessage = 'ROS not connected';
			return;
		}

		const id = generateID();
		pendingSaveId = id;
		statusMessage = 'Sending JSON save request…';

		requestTopic.publish({
			data: JSON.stringify({
			id,
			op: 'save_json',
			file_name: 'Waypoints.json',
			json_text: buildJson()
			})
		});
	}

	async function onMapClick(event: MouseEvent) {
		const xOriginDirection = positiveXDirection;
		const yOriginDirection = positiveYDirection;
		if (mode === 'manual') {
			const clickPosition = clickToNaturalPosition(event);
			if (!clickPosition) return;
			currentClickPosition = clickPosition;
			addManualWaypoint(clickPosition);
			return;
		}
		if (mode ==='origin'){
			if(xOriginDirection === 'unselected' || yOriginDirection === 'unselected'){
				statusMessage = 'Please select both X and Y positive directions for origin mode.';
				return;
			}
			const clickPosition = clickToNaturalPosition(event);
			if(!clickPosition) return;
			addManualOrigin(clickPosition);
			return;
		}
			const clickPosition = clickToNaturalPosition(event);
			if(!clickPosition) return;
			currentClickPosition = clickPosition;
			contour = [];
			centroid = null;
			sampleImageHexadecimalColor = '';

			if (!clickPosition) return;

			statusMessage = `${mode} @ (${clickPosition.x}, ${clickPosition.y})…`;

<<<<<<< HEAD
			const extractionMode = mode === 'waypoint' ? 'border' : mode;

			const request = ({
				op: 'extract_feature',
				mode: extractionMode,
				imageID: mapImageId,
				sampleXPosition: clickPosition.x,
				sampleYPosition: clickPosition.y,
				hueTolerance: defaultHueTolerance,
				saturationTolerance: defaultSaturationTolerance,
				valueTolerance: defaultValueTolerance,
				xOriginDirectionection: xOriginDirection,
				yOriginDirectionection: yOriginDirection
			});
=======
			const request = ({
				op: 'extract_feature',
				mode, // 'waypoint' | 'border' | 'origin' (protocol value)
				image_id: mapImageId,
				sample_x: click_position.x,
				sample_y: click_position.y,
				tol_h: defaultHueTolerance,
				tol_s: defaultSaturationTolerance,
				tol_v: defaultValueTolerance,
				x_direction: x_dir,
				y_direction: y_dir
			});

>>>>>>> e2ab3e4e (fix(map-gui): simplified arena mapping publisher and subscriber)
			const id = generateID();
			if (requestTopic) {
				requestTopic.publish({
					data: JSON.stringify({ id, ...request})
				});
			}
			else {
<<<<<<< HEAD
				statusMessage = 'ROS not connected';
				return;
			}

	}

	function updateScale() {
		const squares = document.getElementById("squares") as HTMLInputElement;
		const gridSpacing = document.getElementById("gridSpacing") as HTMLInputElement;

		const nSquares = squares.valueAsNumber;
		const metersPerSquare = gridSpacing.valueAsNumber;

		if (
			mapHeight > 0 &&
			Number.isFinite(nSquares) && nSquares > 0 &&
			Number.isFinite(metersPerSquare) && metersPerSquare > 0
		) {
			// pixels per meter
			scale = mapHeight / (nSquares * metersPerSquare);
			scale = parseFloat(scale.toFixed(6));
		}
	}

=======
				new Error('ROS not connected');
			};
		}

	}

	function update_scale() {
		const squares = document.getElementById("squares") as HTMLInputElement;
		const grid_spacing = document.getElementById("grid_spacing") as HTMLInputElement;

		if (squares.valueAsNumber && grid_spacing.valueAsNumber) {
			scale = map_height / squares.valueAsNumber * grid_spacing.valueAsNumber;
			scale = parseFloat(scale.toFixed(2));
		}
	};

>>>>>>> e2ab3e4e (fix(map-gui): simplified arena mapping publisher and subscriber)
</script>
<ScrollArea orientation="vertical" class="relative flex h-full w-full">
	<div class="wrap">
		<div class="topbar">
			<div class="status">Status: {statusMessage}</div>

			<div class="controls flex ">
				<button class:active={mode === 'waypoint'} onclick={() => (mode = 'waypoint')}>
					Waypoint mode
				</button>
				<button class:active={mode === 'border'} onclick={() => (mode = 'border')}>
					Border mode
				</button>
				<button class:active={mode === 'origin'} onclick={() => (mode = 'origin')}>
					Origin mode
				</button>
				<button class:active={mode === 'manual'} onclick={() => (mode = 'manual')}>
					Manual mode
				</button>
				<button type="button" class="danger" onclick={clearWaypoints} disabled={waypoints.length === 0}>
					Clear table
				</button>

				<button type="button" onclick={saveJsonToScripts} disabled={waypoints.length === 0}>
					Save YAML
				</button>
			</div>
		</div>

		<div class="row">
			<div class="frame">
				<!-- svelte-ignore a11y_click_events_have_key_events -->
				<!-- svelte-ignore a11y_no_noninteractive_element_interactions -->
				<img
					bind:this={imageElement}
					src={mapImageId}
					width="700"
					alt="map"
					class="map"
					onclick={onMapClick}
					onload={() => {
						if (!imageElement) return;
						svgViewBox = `0 0 ${imageElement.naturalWidth} ${imageElement.naturalHeight}`;
<<<<<<< HEAD
						mapHeight = imageElement.naturalHeight;
=======
						map_height = imageElement.naturalHeight;
>>>>>>> e2ab3e4e (fix(map-gui): simplified arena mapping publisher and subscriber)
					}}
				/>
			</div>

			<!-- Preview -->
			<svg class="preview" viewBox={svgViewBox} preserveAspectRatio="none" aria-hidden="true">
				{#if borderContour.length > 0}
					<polygon
<<<<<<< HEAD
						points={borderContour.map(([x, y]) => `${x}, ${y}`).join(' ')}
=======
						points={borderContour.map(([x, y]) => `${x},${y}`).join(' ')}
>>>>>>> e2ab3e4e (fix(map-gui): simplified arena mapping publisher and subscriber)
						fill="none"
						stroke="lime"
						stroke-width="2"
						opacity="0.9"
					/>
				{/if}
				
				{#each origins as origin (origin.id)}
					<circle cx={origin.centroidX} cy={origin.centroidY} r="6" fill="orange" />
				{/each}
				{#each waypoints as waypoint (waypoint.id)}
					{#if waypoint.contour && waypoint.contour.length > 0}
						<polygon
							points={waypoint.contour.map(([x, y]) => `${x},${y}`).join(' ')}
							stroke="lime"
							stroke-width="2"
							opacity="0.9"
						/>
					{/if}
				{/each}

				{#each waypoints as waypoint (waypoint.id)}
					{@const yawDirection = drawRotatedRectangle(waypoint)}
					<circle cx={waypoint.centroidX} cy={waypoint.centroidY} r="6" fill="red" />
					<line
						x1={yawDirection.yawX1Coordinate}
						y1={yawDirection.yawY1Coordinate}
						x2={yawDirection.yawX2Coordinate}
						y2={yawDirection.yawY2Coordinate}
						stroke="yellow"
						stroke-width="4"
					/>
				{/each}
			</svg>
		</div>

		<!--Scale table-->
		<div class="tableWrap">
			<h2>Scale</h2>
			<table class="tbl" style="width: 50%">
				<thead>
					<tr>
						<th>Grid spacing (m)</th>
						<th>Number of squares (top to bottom)</th>
						<th>Image height (pixels)</th>
						<th>Scale (pixels per meter)</th>
					</tr>
				</thead>
				<tbody>
					<tr>
						<td>
							<input onchange={updateScale} id="gridSpacing" type="number" min="0" step="0.01" value="0.00"/>
						</td>
						<td>
							<input onchange={updateScale} id="squares" type="number" min="0" value="0"/>
						</td>
						<td>
							<p>{mapHeight}</p>
						</td>
						<td>
							<p>{scale}</p>
						</td>
					</tr>
				</tbody>
			</table>
		</div>

		<!--Origin table-->
		<div class="tableWrap">
			<h2>Origin</h2>
			<table class="tbl" style="width:70%">
				<thead>
					<tr>
						<th>Name</th>
						<th>Hex</th>
						<th>Positive X direction</th>
						<th>Origin X</th>
						<th>Positive Y direction</th>
						<th>Origin Y</th>
						<th></th>
					</tr>
				</thead>
				<tbody>
					{#if origins.length === 0}
						<tr>
							<td></td>
							<td></td>
							<td>
								<select name="positive direction" id="p_direction_x">
									<option disabled selected value="unselected">Select direction</option>
									<option value="up" onclick={() => (positiveXDirection = 'up')}>Up</option>
									<option value="down" onclick={() => (positiveXDirection = 'down')}>Down</option>
									<option value="left" onclick={() => (positiveXDirection = 'left')}>Left</option>
									<option value="right" onclick={() => (positiveXDirection = 'right')}>Right</option>
								</select>
							</td>
							<td></td>
							<td>
								<select name="positive direction" id="p_direction_y">
									<option disabled selected value="unselected">Select direction</option>
									<option value="up" onclick={() => (positiveYDirection = 'up')}>Up</option>
									<option value="down" onclick={() => (positiveYDirection = 'down')}>Down</option>
									<option value="left" onclick={() => (positiveYDirection = 'left')}>Left</option>
									<option value="right" onclick={() => (positiveYDirection = 'right')}>Right</option>
								</select>
							</td>
							<td></td>
						</tr>
					{:else}
						{#each origins as origin, i (origin.id)}
						<tr>
							<td> {origin.name} </td>
							<td>
								<span class="chip" style={`background:${origin.hexadecimalColor || '#eee'}`}>
									{origin.hexadecimalColor}
								</span>
							</td>
							<td> {positiveXDirection} </td>
							<td>{(origin.centroidX / scale).toFixed(2)}</td>
							<td> {positiveYDirection} </td>
							<td>{(origin.centroidY / scale).toFixed(2)}</td>
							<td class="right">
								<button type="button" class="danger" onclick={() => deleteOrigin(origin.id)}>
									Delete
								</button>
							</td>
						</tr>
						{/each}
					{/if}
					<tr>
						<td class="right">
							<button type="button" class="danger" onclick={() => calculateOrigin()}>
								Calculate average
							</button>
						</td>
						<td>{xOriginAverage / scale}</td>
						<td>{yOriginAverage / scale}</td>
					</tr>
				</tbody>
			</table>
		</div>

		<!-- Big table -->
		<div class="tableWrap">
			<h2>Waypoints</h2>
			<table class="tbl" style="min-width: 900px;">
				<thead>
					<tr>
						<th>#</th>
						<th>Name</th>
						<th>Hex</th>
						<th>Waypoint X</th>
						<th>Relative X</th>
						<th>Waypoint Y</th>
						<th>Relative Y</th>
						<th>Yaw</th>
						<th></th>
					</tr>
				</thead>
				<tbody>
					{#if waypoints.length === 0}
						<tr>
							<td colspan="8" class="empty">No waypoints yet — switch to Waypoint mode and click markers.</td>
						</tr>
					{:else}
						{#each waypoints as waypoint, i (waypoint.id)}
							<tr>
								<td>{i + 1}</td>
								<td>
									<input
										class="nameInput"
										value={waypoint.name}
										oninput={(event) =>
											updateName(waypoint.id, (event.target as HTMLInputElement).value)}
									/>
								</td>
								<td>
									<span class="chip" style={`background:${waypoint.hexadecimalColor || '#eee'}`}>
										{waypoint.hexadecimalColor}
									</span>
								</td>
								<td>{(waypoint.centroidX / scale).toFixed(2)}</td>
								<td>{(waypoint.relativeX / scale).toFixed(2)}</td>
								<td>{(waypoint.centroidY / scale).toFixed(2)}</td>
								<td>{(waypoint.relativeY / scale).toFixed(2)}</td>
								<td>
									<input
										class="yawInput" type="number" min="0" max="360"step="1"
										value={waypoint.yaw} oninput={(event) => updateYaw(waypoint.id, Number((event.target as HTMLInputElement).value))}
									/>
								</td>

								<td class="right">
									<button type="button" class="danger" onclick={() => deleteWaypoint(waypoint.id)}>
										Delete
									</button>
								</td>
							</tr>
						{/each}
					{/if}
				</tbody>
			</table>
		</div>

		<!-- YAML preview -->
		<div class="jsonWrap">
			<h2>JSON preview</h2>
			<textarea class="jsonBox" readonly value={buildJson()}></textarea>
		</div>
	</div>
</ScrollArea>
<style>

	.topbar {
		display: flex;
		flex-direction: column;
		margin-bottom: 12px;
	}
	.controls {
		display: flex;
		gap: 8px;
	}
 
	button {
		border: 1px solid #444;
		padding: 6px 10px;
		border-radius: 8px;
		background: #111;
		color: #eee;
		cursor: pointer;
	}
	button.active {
		border-color: #7dd3fc;
		box-shadow: 0 0 0 2px rgba(125, 211, 252, 0.2);
	}
	button.danger {
		border-color: #ef4444;
	}
	button:disabled {
		opacity: 0.5;
		cursor: not-allowed;
	}

	.row {
		display: flex;
		gap: 16px;
		align-items: flex-start;
	}
	.frame {
		width: 700px;
	}
	.map {
		display: block;
		width: 100%;
		height: auto;
	}
	.preview {
		width: 700px;
		height: auto;
		border: 1px solid #333;
	}

	.tableWrap {
		margin-top: 16px;
		/*border: 1px solid #333;*/
		max-height: 340px; /* big + scrollable */
		overflow: auto;
	}

	.tbl {
		width: 100%;
		border-collapse: collapse;
		min-width: 500px;
	}
	.tbl th,
	.tbl td {
		border: 1px solid #333;
		padding: 6px 8px;
		font-size: 13px;
		white-space: nowrap;
	}
	.tbl th {
		text-align: left;
		opacity: 0.9;
		position: sticky;
		top: 0;
		background: #0b0b0b;
	}

	.nameInput {
		width: 140px;
		border: 1px solid #333;
		border-radius: 6px;
		background: #0b0b0b;
		color: #eee;
		padding: 4px 6px;
	}

	.chip {
		display: inline-block;
		padding: 2px 6px;
		border-radius: 6px;
		color: #111;
		border: 1px solid #333;
		min-width: 84px;
		text-align: center;
	}

	.jsonWrap {
		margin-top: 14px;
	}

	h2 {
		font-weight: 600;
		margin-bottom: 6px;
	}

	.jsonBox {
		width: 100%;
		min-height: 180px;
		border: 1px solid #333;
		border-radius: 8px;
		background: #0b0b0b;
		color: #eee;
		padding: 10px;
		font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, 'Liberation Mono',
			'Courier New', monospace;
		font-size: 12px;
	}
</style>
