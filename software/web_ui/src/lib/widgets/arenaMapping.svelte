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
	//KEY KAD!!
	export {
		widget_name as name,
		widget_description as description,
		widget_group as group,
		is_ros_dependent as isRosDependant,
		widget_settings as settings
	};
</script>

<script lang="ts">
	import * as ROSLIB from 'roslib';
	import { getRosConnection as getRosConnection } from '$lib/scripts/rosBridge.svelte';
	//import Button from "$lib/components/ui/button/button.svelte";
	import { ScrollArea } from '$lib/components/ui/scroll-area/index';
	import { DoubleArrowDown } from 'svelte-radix';

	//Set these as the only valid string values 
	type Mode = 'waypoint' | 'border';
	type rosStringMessage = { data: string };
	//Data modle for the waypoints for the YAML output and table 
	type waypointRow = {
		id: string;
		name: string;
		hexadecimalColor: string;
		clickX: number;
		clickY: number;
		centroidX: number;
		centroidY: number;
		yaw: number;
		contour: number[][];
	};

	let imageElement: HTMLImageElement | null = null;

	const mapImageId = 'Cropped_ARCH_2025_Autonomous_map_2.png';
	const mapImageUrl = `http://localhost:8000/${mapImageId}`;

	//Ros topics to listen to requests and reply 
	const requestTopicName = '/map_editor/request';
	const responseTopicName = '/map_editor/response';

	const defaultRequestTimeoutMilliseconds = 6000;

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
	let svg_view_box = $state('0 0 1 1');

	// stored waypoints
	let waypoints = $state<waypointRow[]>([]);
	let border_contour = $state<number[][]>([]);
	// ROS request/response 
	let requestTopic: ROSLIB.Topic<rosStringMessage> | null = null;
	let response_topic: ROSLIB.Topic<rosStringMessage> | null = null;

	const pendingRequests = new Map<
		string,
		{ resolve: (response: any) => void; timeoutHandle: any }
	>();

	//Generates the ID tp send to the backend 
	function generateId() {
		return typeof crypto !== 'undefined' && 'randomUUID' in crypto
			? crypto.randomUUID()
			: `${Date.now()}-${Math.random().toString(16).slice(2)}`;
	}
	
	function clamp(value: number, min_value: number, max_value: number) {
		return Math.max(min_value, Math.min(max_value, value));
	}
	//Converts the click on the image to the pixle coordinates on the image 
	function clickToNatrualPosition(event: MouseEvent) {
		if (!imageElement) return null;

		const image_rectangle = imageElement.getBoundingClientRect();
		const displayX = event.clientX - image_rectangle.left;
		const displayY = event.clientY - image_rectangle.top;

		const natrualX = Math.round(displayX * (imageElement.naturalWidth / image_rectangle.width));
		const natrualY = Math.round(displayY * (imageElement.naturalHeight / image_rectangle.height));

		return {
			x: clamp(natrualX, 0, imageElement.naturalWidth - 1),
			y: clamp(natrualY, 0, imageElement.naturalHeight - 1)
		};
	}
	// Function that allows the front end to talk to the back end 
	function sendRequest(payload: any, timeoutMilliseconds = defaultRequestTimeoutMilliseconds): Promise<any> {
		if (!requestTopic) return Promise.reject(new Error('ROS not connected'));

		const id = generateId();
		return new Promise((resolve, reject) => {
			const timeoutHandle = setTimeout(() => {
				pendingRequests.delete(id);
				reject(new Error(`Timeout waiting for response (${id})`));
			}, timeoutMilliseconds);

			pendingRequests.set(id, { resolve, timeoutHandle });

			requestTopic!.publish({
				data: JSON.stringify({ id, ...payload })
			});

		});
	}
	// creates the waypoint names
	function nextWaypointName() {
		const waypointNumber = waypoints.length + 1;
		return `WP${String(waypointNumber)}`;
	}
	//extracts the coordinates from the centroid 
	function addWaypointFromResponse(response: any, extractedContour: number [][]) {
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
				id: generateId(),
				name: nextWaypointName(),
				hexadecimalColor,
				clickX,
				clickY,
				centroidX,
				centroidY,
				yaw:0,
				contour:waypointContour
			}
		];
	}

	function deleteWaypoints(id: string) {
		waypoints = waypoints.filter((waypoint) => waypoint.id !== id);
	}

	function clearWaypoints() {
		waypoints = [];
	}
	function updateYaw(id: string, yawDeg: number) {
		const yaw = Number.isFinite(yawDeg) ? yawDeg : 0;
		waypoints = waypoints.map((w) => (w.id === id ? { ...w, yaw } : w));
	}

	function updateName(id: string, name: string) {
		waypoints = waypoints.map((waypoint) => (waypoint.id === id ? { ...waypoint, name } : waypoint));
	}

	function buildYaml(): string {
		const lines: string[] = [];
		
		lines.push('waypoints:');

		for (const waypoint of waypoints) {
			const yawRadians = ((waypoint.yaw)*Math.PI)/180;
			lines.push(`- name: ${waypoint.name}`);
			lines.push(`	x: ${waypoint.centroidX}`);
			lines.push(`	y: ${waypoint.centroidY}`);
			lines.push(`	yaw: ${yawRadians}`)
		}

		return lines.join('\n');
	}
	  
	function drawRotatedRectangle(waypoint:waypointRow){
			const arrowLength = 25; 

			const yawDegrees = ((waypoint.yaw-90) * Math.PI) / 180;

			const yawX1Coordinate = waypoint.centroidX;
			const yawY1Coordinate = waypoint.centroidY; 

			const yawX2Coordinate = yawX1Coordinate + Math.cos(yawDegrees) * arrowLength;
			const yawY2Coordinate = yawY1Coordinate + Math.sin(yawDegrees) * arrowLength;
			return{yawX1Coordinate,yawX2Coordinate, yawY1Coordinate, yawY2Coordinate};
	}

// Function to help translate the difference in case vairables from the back end to front end
	function getSampleHex(response: any): string {
		return String(
			response?.sample_image_hex ??       
			response?.sampleImageHex ??      
			response?.sample_image_hexadecimal_color ?? 
			''
		);
	}
// Function that sets the file name of the saved file created in buildYaml()
	async function saveYamlToScripts() {
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
	}

	async function onMapClick(event: MouseEvent) {
		const click_position = clickToNatrualPosition(event);

		currentClickPosition = click_position;
		contour = [];
		centroid = null;
		sampleImageHexadecimalColor = '';

		if (!click_position) return;

		statusMessage = `${mode === 'waypoint' ? 'Waypoint' : 'Border'} @ (${click_position.x}, ${click_position.y})…`;

		try {
			const response = await sendRequest({
				op: 'extract_feature',
				mode, // 'waypoint' | 'border' (protocol value)
				image_id: mapImageId,
				sample_x: click_position.x,
				sample_y: click_position.y,
				tol_h: defaultHueTolerance,
				tol_s: defaultSaturationTolerance,
				tol_v: defaultValueTolerance
			});

			if (!response?.ok) {
				statusMessage = response?.message ?? 'Failed';
				return;
			}

			if (mode === 'border') {
				border_contour = Array.isArray(response.contour) ? response.contour : [];
			}

			sampleImageHexadecimalColor = getSampleHex(response);
			centroid = Array.isArray(response.centroid) ? (response.centroid as [number, number]) : null;
			contour = Array.isArray(response.contour) ? response.contour : [];

			if (mode === 'waypoint') {
				addWaypointFromResponse(response, contour);
				statusMessage = centroid
					? `Waypoint added — centroid (${centroid[0]}, ${centroid[1]}) ${sampleImageHexadecimalColor}`
					: `Waypoint added ${sampleImageHexadecimalColor}`;
			} else {
				statusMessage = `Border OK ${sampleImageHexadecimalColor}`;
			}
		} catch (error: any) {
			statusMessage = `Error: ${error?.message ?? String(error)}`;
		}
	}

	$effect(() => {
		const ros_connection = getRosConnection();

		if (!ros_connection) {
			response_topic?.unsubscribe();
			requestTopic = null;
			response_topic = null;

			pendingRequests.forEach((pending_entry) => clearTimeout(pending_entry.timeoutHandle));
			pendingRequests.clear();
			return;
		}

		requestTopic = new ROSLIB.Topic({
			ros: ros_connection,
			name: requestTopicName,
			messageType: 'std_msgs/msg/String'
		});

		response_topic = new ROSLIB.Topic({
			ros: ros_connection,
			name: responseTopicName,
			messageType: 'std_msgs/msg/String'
		});

		response_topic.subscribe((message: any) => {
			let response: any;

			try {
				response = JSON.parse(message.data);
			} catch {
				return;
			}

			const id = response?.id;
			const pending_entry = id ? pendingRequests.get(id) : null;
			if (!pending_entry) return;

			clearTimeout(pending_entry.timeoutHandle);
			pendingRequests.delete(id);
			pending_entry.resolve(response);
		});

		return () => {
			response_topic?.unsubscribe();
			requestTopic = null;
			response_topic = null;
		};
	});
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

				<button type="button" class="danger" onclick={clearWaypoints} disabled={waypoints.length === 0}>
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
						svg_view_box = `0 0 ${imageElement.naturalWidth} ${imageElement.naturalHeight}`;
					}}
				/>
			</div>

			<!-- Preview -->
			<svg class="preview" viewBox={svg_view_box} preserveAspectRatio="none" aria-hidden="true">
				{#if border_contour.length > 0}
					<polygon
						points={border_contour.map(([x, y]) => `${x},${y}`).join(' ')}
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

		<!-- Big table -->
		<div class="tableWrap">
			<table class="tbl">
				<thead>
					<tr>
						<th>#</th>
						<th>Name</th>
						<th>Hex</th>
						<th>Waypoint X</th>
						<th>Waypoint Y</th>
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
								<td><input value={waypoint.name}/></td>
								<td>
									<span class="chip" style={`background:${waypoint.hexadecimalColor || '#eee'}`}>
										{waypoint.hexadecimalColor}
									</span>
								</td>
								<td>{waypoint.centroidX}</td>
								<td>{waypoint.centroidY}</td>
								<td><input
									type="number" value={waypoint.yaw} step="1"
									oninput={(e) => updateYaw(waypoint.id, (e.currentTarget as HTMLInputElement).valueAsNumber)}/></td>
								<td class="right">
									<button type="button" class="danger" onclick={() => deleteWaypoints(waypoint.id)}>
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
			<div class="yamlTitle">YAML preview</div>
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
		border: 1px solid #333;
		max-height: 340px; /* big + scrollable */
		overflow: auto;
	}

	.tbl {
		width: 100%;
		border-collapse: collapse;
		min-width: 900px;
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
	.yamlTitle {
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
