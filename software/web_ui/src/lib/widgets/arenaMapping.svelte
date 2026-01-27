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
	//import { response } from 'express';

	type Mode = 'waypoint' | 'border' | 'origin';
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

	let imageElement: HTMLImageElement | null = null;

	const mapImageId = 'Cropped_ARCH_2025_Autonomous_map_2.png';
	const mapImageUrl = `http://localhost:8000/${mapImageId}`;

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
	// 0 -> selecting waypoint
	// 1 -> selecting arrow
	let lastID = $state('');

	//stored origin
	let origins = $state<OriginRow[]>([]);

	//stored arrow
	let arrows = $state<ArrowRow[]>([]);

	let borderContour = $state<number[][]>([]);

	// ROS request/response plumbing
	let requestTopic: ROSLIB.Topic<rosStringMessage> | null = null;
	let responseTopic: ROSLIB.Topic<rosStringMessage> | null = null;

	//scale
	let scale = $state(1);
	let map_height = $state(0);

	//origin
	let positive_x_direction = $state<Direction>('unselected');
	let positive_y_direction = $state<Direction>('unselected');
	let x_origin = $state(0);
	let y_origin = $state(0);

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
		});
		x_origin = total_x / origins.length;
		y_origin = total_y / origins.length;

		waypoints.forEach((waypoint) => {
			switch(positive_x_direction) {
				case "left":
					waypoint.relativeX = x_origin - waypoint.centroidX;
					break;
				case "right":
					waypoint.relativeX = waypoint.centroidX - x_origin;
					break;
				case "down":
					waypoint.relativeX = waypoint.centroidY - y_origin;
					break;
				case "up":
					waypoint.relativeX = y_origin - waypoint.centroidY;
			}
			switch(positive_y_direction) {
				case "left":
					waypoint.relativeY = x_origin - waypoint.centroidX;
					break;
				case "right":
					waypoint.relativeY = waypoint.centroidX - x_origin;
					break;
				case "down":
					waypoint.relativeY = waypoint.centroidY - y_origin;
					break;
				case "up":
					waypoint.relativeY = y_origin - waypoint.centroidY;
			}
		});
	}

	function calculate_angle() {
		waypoints.forEach((waypoint, w) => {
			const delta_x = waypoint.centroidX - arrows[w].centroidX;
			const delta_y = waypoint.centroidY - arrows[w].centroidY;

			if (delta_x == 0) {
				if (waypoint.centroidY <= arrows[w].centroidY) {
					waypoint.yaw = Math.PI;
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

	function generateID() {
		lastID = typeof crypto !== 'undefined' && 'randomUUID' in crypto
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

	// creates the waypoint names
	function nextWaypointName() {
		const waypointNumber = waypoints.length + 1;
		return `WP${String(waypointNumber)}`;
	}

	function next_origin_name() {
		const origin_number = origins.length + 1;
		return `O${String(origin_number)}`;
	}

	function next_arrow_name() {
		const arrow_number = arrows.length + 1;
		return `AR${String(arrow_number)}`;
	}

	function add_waypoint_from_response(response: any, extractedContour: number [][]) {
		if (!response?.ok) return;
		if (!Array.isArray(response.centroid) || response.centroid.length !== 2) return;

		const centroidX = Number(response.centroid[0]);
		const centroidY = Number(response.centroid[1]);

		// Prefer server echo; fall back to local currentClickPosition.
		const clickX = Number(response.sample_x ?? currentClickPosition?.x);
		const clickY = Number(response.sample_y ?? currentClickPosition?.y);

		if (
			!Number.isFinite(clickX) ||
			!Number.isFinite(clickY) ||
			!Number.isFinite(centroidX) ||
			!Number.isFinite(centroidY)
		) {
			return;
		}

		const hexadecimalColor = getSampleHex(response);
		const waypointContour = extractedContour;


		waypoints = [
			...waypoints,
			{
				id: generateID(),
				name: nextWaypointName(),
				hexadecimalColor: hexadecimalColor,
				clickX,
				clickY,
				centroidX,
				centroidY,
				relativeX: centroidX,
				relativeY: centroidY,
				yaw:0,
				contour:waypointContour
			}
		];
	}

	function addAngleFromResponse(response: any) {
		if (!response?.ok) return;
		if (!Array.isArray(response.centroid) || response.centroid.length !== 2) return;

		const centroidX = Number(response.centroid[0]);
		const centroidY = Number(response.centroid[1]);

		// Prefer server echo; fall back to local currentClickPosition.
		const clickX = Number(response.sample_x ?? currentClickPosition?.x);
		const clickY = Number(response.sample_y ?? currentClickPosition?.y);

		if (
			!Number.isFinite(clickX) ||
			!Number.isFinite(clickY) ||
			!Number.isFinite(centroidX) ||
			!Number.isFinite(centroidY)
		) {
			return;
		}

		const hexadecimalColor = String(response.sample_image_hex ?? '');

		arrows = [
			...arrows,
			{
				id: generateID(),
				name: next_arrow_name(),
				hexadecimalColor,
				clickX,
				clickY,
				centroidX,
				centroidY
			}
		];
	}

	function add_origin_from_response(response: any) {
		if (!response?.ok) return;
		if (!Array.isArray(response.centroid) || response.centroid.length !== 2) return;

		const centroidX = Number(response.centroid[0]);
		const centroidY = Number(response.centroid[1]);

		// Prefer server echo; fall back to local currentClickPosition.
		const clickX = Number(response.sample_x ?? currentClickPosition?.x);
		const clickY = Number(response.sample_y ?? currentClickPosition?.y);

		if (
			!Number.isFinite(clickX) ||
			!Number.isFinite(clickY) ||
			!Number.isFinite(centroidX) ||
			!Number.isFinite(centroidY)
		) {
			return;
		}

		const hexadecimalColor = String(response.sample_image_hex ?? '');

		origins = [
			...origins,
			{
				id: generateID(),
				name: nextWaypointName(),
				hexadecimalColor,
				clickX,
				clickY,
				centroidX,
				centroidY
			}
		];
	}

	function delete_waypoint(id: string) {
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

		return lines.join('\n');
	}

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
	}


// Function to help translate the difference in case variables from the back end to front end
	function getSampleHex(response: any): string {
		return String(
			response?.sample_image_hex ??
			response?.sampleImageHex ??
			response?.sampleImageHexadecimalColor ??
			''
		);
	}
// Function that sets the file name of the saved file created in buildYaml()
	async function saveYamlToScripts() {
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
		}

		if (mode === 'origin' && x_dir == 'unselected' && y_dir == 'unselected') {
			statusMessage = "Assign positive direction for x and y";
		}
		else {
			const click_position = clickToNaturalPosition(event);

			currentClickPosition = click_position;
			contour = [];
			centroid = null;
			sampleImageHexadecimalColor = '';

			if (!click_position) return;

			statusMessage = `${mode} @ (${click_position.x}, ${click_position.y})…`;

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

			const id = generateID();
			if (requestTopic) {
				requestTopic.publish({
					data: JSON.stringify({ id, ...request})
				});
			}
			else {
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
				<button type="button" class="danger" onclick={clear_waypoints} disabled={waypoints.length === 0}>
					Clear table
				</button>

				<button type="button" onclick={saveYamlToScripts} disabled={waypoints.length === 0}>
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
					src={mapImageUrl}
					width="700"
					alt="map"
					class="map"
					onclick={onMapClick}
					onload={() => {
						if (!imageElement) return;
						svgViewBox = `0 0 ${imageElement.naturalWidth} ${imageElement.naturalHeight}`;
						map_height = imageElement.naturalHeight;
					}}
				/>
			</div>

			<!-- Preview -->
			<svg class="preview" viewBox={svgViewBox} preserveAspectRatio="none" aria-hidden="true">
				{#if borderContour.length > 0}
					<polygon
						points={borderContour.map(([x, y]) => `${x},${y}`).join(' ')}
						fill="none"
						stroke="lime"
						stroke-width="2"
						opacity="0.9"
					/>
				{/if}

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
							<input onchange={update_scale} id="grid_spacing" type="number" min="0" step="0.01" value="0.00"/>
						</td>
						<td>
							<input onchange={update_scale} id="squares" type="number" min="0" value="0"/>
						</td>
						<td>
							<p>{map_height}</p>
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
									<option value="up" onclick={() => (positive_x_direction = 'up')}>Up</option>
									<option value="down" onclick={() => (positive_x_direction = 'down')}>Down</option>
									<option value="left" onclick={() => (positive_x_direction = 'left')}>Left</option>
									<option value="right" onclick={() => (positive_x_direction = 'right')}>Right</option>
								</select>
							</td>
							<td></td>
							<td>
								<select name="positive direction" id="p_direction_y">
									<option disabled selected value="unselected">Select direction</option>
									<option value="up" onclick={() => (positive_y_direction = 'up')}>Up</option>
									<option value="down" onclick={() => (positive_y_direction = 'down')}>Down</option>
									<option value="left" onclick={() => (positive_y_direction = 'left')}>Left</option>
									<option value="right" onclick={() => (positive_y_direction = 'right')}>Right</option>
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
							<td> {positive_x_direction} </td>
							<td>{(origin.centroidX / scale).toFixed(2)}</td>
							<td> {positive_y_direction} </td>
							<td>{(origin.centroidY / scale).toFixed(2)}</td>
							<td class="right">
								<button type="button" class="danger" onclick={() => delete_origin(origin.id)}>
									Delete
								</button>
							</td>
						</tr>
						{/each}
					{/if}
					<tr>
						<td class="right">
							<button type="button" class="danger" onclick={() => calculate_origin()}>
								Calculate average
							</button>
						</td>
						<td>{x_origin / scale}</td>
						<td>{y_origin / scale}</td>
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
								<td>{(waypoint.yaw * (180 / Math.PI)).toFixed(2)}</td>
								<td class="right">
									<button type="button" class="danger" onclick={() => delete_waypoint(waypoint.id)}>
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
		<div class="yamlWrap">
			<h2>YAML preview</h2>
			<textarea class="yamlBox" readonly value={buildYaml()}></textarea>
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

	.yamlWrap {
		margin-top: 14px;
	}

	h2 {
		font-weight: 600;
		margin-bottom: 6px;
	}

	.yamlBox {
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
