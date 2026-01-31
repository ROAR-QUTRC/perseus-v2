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

	let pendingSaveId: string | null = null; 

	// ROS request/response plumbing
	let requestTopic: ROSLIB.Topic<rosStringMessage> | null = null;
	let responseTopic: ROSLIB.Topic<rosStringMessage> | null = null;

	//scale
	let scale = $state(1);
	let mapHeight = $state(0);

	//origin
	let positiveXDirection = $state<Direction>('up');
	let positiveYDirection = $state<Direction>('right');
	let xOriginAverage = $state(0);
	let yOriginAverage = $state(0);

	$effect(() => {
		const rosConnection = getRosConnection();
		if (rosConnection) {
			requestTopic = new ROSLIB.Topic({
				ros: rosConnection,
				name: requestTopicName,
				messageType: 'std_msgs/msg/String'
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

	const onResponseMessage = (message: rosStringMessage) => {
		const response: any = JSON.parse(message.data);
		if (pendingSaveId && response?.id === pendingSaveId) {
			pendingSaveId = null;

			statusMessage = response?.ok
			? `Saved: ${response.saved_path ?? 'OK'}`
			: `Save failed: ${response?.message ?? 'Unknown error'}`;

			return;
		}
		if (response?.op === 'save_json') {
			statusMessage = response?.ok
			? `Saved: ${response.saved_path ?? 'OK'}`
			: `Save failed: ${response?.message ?? 'Unknown error'}`;
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
			if (waypointToggle == 0) {
				addWaypointFromRespomse(response, contour);
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
				calculateAngle();
			}
		}
		 else if (mode === 'origin') {
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
		}
		return {
			relativeX: yOriginAverage - centroidY,
			relativeY: centroidX - xOriginAverage
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
		const rel = toRelative(clickX, clickY);

		waypoints = [
			...waypoints,
			{
				id: generateID(),
				name: nextWaypointName(),
				hexadecimalColor: '#ffffff', // manual marker colour
				clickX,
				clickY,
				centroidX: clickX,
				centroidY: clickY,
				relativeX: rel.relativeX,
				relativeY: rel.relativeY,
				yaw: 0,          // store yaw as DEGREES (0=N, 90=W, 180=S, 270=E)
				contour: []      // no contour in manual mode so left blank 
			}
		];

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



	function addWaypointFromRespomse(response: any, extractedContour: number [][]) {
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

		const hexadecimalColor = getSampleHex(response);
		const waypointContour = extractedContour;
		const rel = toRelative(centroidX, centroidY);

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
				relativeX: rel.relativeX,
				relativeY: rel.relativeY,
				yaw: 0,
				contour: waypointContour
			}
		];
	}

	function addAngleFromResponse(response: any) {
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

		arrows = [
			...arrows,
			{
				id: generateID(),
				name: nextArrowName(),
				hexadecimalColor,
				clickX,
				clickY,
				centroidX,
				centroidY
			}
		];
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

	function wrapPi(rad: number) {
		return Math.atan2(Math.sin(rad), Math.cos(rad));
	}
	function roundJsonOutput(n: number, dp: number) {
		return Number.isFinite(n) ? Number(n.toFixed(dp)) : 0;
	}
	function buildJson(): string {
		const payload = waypoints.map((w) => {
			const xRelativeCoordinate = w.relativeX / scale;
			const yRelativeCoordinate = w.relativeY / scale;

			// yaw stored as DEGREES in UI but output yaw in RADIANS for navigation
			const yawOutputRadians = wrapPi((w.yaw * Math.PI) / 180);

			return {
			name: w.name,
			x: roundJsonOutput(xRelativeCoordinate, 3),
			y: roundJsonOutput(yRelativeCoordinate, 3),
			yaw: roundJsonOutput(yawOutputRadians, 6)
			};
		});

		return JSON.stringify(payload, null, 2);
		}

	function drawRotatedRectangle(waypoint: WaypointRow) {
		const arrowLength = 25;

		// Same convention as JSON: N=0, E=-pi/2, W=+pi/2
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

			const request = ({
				op: 'extract_feature',
				mode, // 'waypoint' | 'border' | 'origin' | 'manual'
				image_id: mapImageId,
				sampleXPosition: clickPosition.x,
				sampleYPosition: clickPosition.y,
				hueTolerance: defaultHueTolerance,
				saturationTolerance: defaultSaturationTolerance,
				valueTolerance: defaultValueTolerance,
				xOriginDirectionection: xOriginDirection,
				yOriginDirectionection: yOriginDirection
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

	function updateScale() {
		const squares = document.getElementById("squares") as HTMLInputElement;
		const gridSpacing = document.getElementById("gridSpacing") as HTMLInputElement;

		if (squares.valueAsNumber && gridSpacing.valueAsNumber) {
			scale = mapHeight / squares.valueAsNumber * gridSpacing.valueAsNumber;
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
				<button class:active={mode === 'manual'} onclick={() => (mode = 'manual')}>
					Manual mode
				</button>
				<button type="button" class="danger" onclick={clearWaypoints} disabled={waypoints.length === 0}>
					Clear table
				</button>

				<button type="button" onclick={saveJsonToScripts} disabled={waypoints.length === 0}>
					Save JSON
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
						mapHeight = imageElement.naturalHeight;
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

		<!-- JSON preview -->
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
