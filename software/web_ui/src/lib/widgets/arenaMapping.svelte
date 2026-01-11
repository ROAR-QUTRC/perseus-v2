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

	// Keep the widget framework’s expected export names via aliasing.
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
	import { getRosConnection as get_ros_connection } from '$lib/scripts/rosBridge.svelte';
	import Button from "$lib/components/ui/button/button.svelte";
	import { ScrollArea } from '$lib/components/ui/scroll-area/index';
	import { Square } from 'svelte-radix';

	type Mode = 'waypoint' | 'border' | 'origin';
	type ros_string_message = { data: string };

	type WaypointRow = {
		id: string;
		name: string;
		hexadecimal_color: string;
		click_x: number;
		click_y: number;
		centroid_x: number;
		centroid_y: number;
		yaw: number;
	};

	let image_element: HTMLImageElement | null = null;

	const map_image_id = 'Cropped_ARCh_2025_Autonomous_map.png';
	const map_image_url = `http://localhost:8000/${map_image_id}`;

	const request_topic_name = '/map_editor/request';
	const response_topic_name = '/map_editor/response';

	const default_request_timeout_milliseconds = 6000;

	// NOTE: These are protocol field names expected by your backend; keeping as-is avoids breaking integration.
	const default_hue_tolerance = 10;
	const default_saturation_tolerance = 60;
	const default_value_tolerance = 60;

	let status_message = $state('Ready');
	let mode = $state<Mode>('waypoint');

	// last response visuals
	let contour = $state<number[][]>([]);
	let current_click_position = $state<{ x: number; y: number } | null>(null);
	let centroid = $state<[number, number] | null>(null);
	let sample_image_hexadecimal_color = $state<string>('');
	let svg_view_box = $state('0 0 1 1');

	// stored waypoints
	let waypoints = $state<WaypointRow[]>([]);
	let border_contour = $state<number[][]>([]);

	// ROS request/response plumbing
	let request_topic: ROSLIB.Topic<ros_string_message> | null = null;
	let response_topic: ROSLIB.Topic<ros_string_message> | null = null;

	//scale
	let scale = $state(0);
	let map_height = $state(0);

	const pending_requests = new Map<
		string,
		{ resolve: (response: any) => void; timeout_handle: any }
	>();

	function generate_id() {
		return typeof crypto !== 'undefined' && 'randomUUID' in crypto
			? crypto.randomUUID()
			: `${Date.now()}-${Math.random().toString(16).slice(2)}`;
	}

	function clamp(value: number, min_value: number, max_value: number) {
		return Math.max(min_value, Math.min(max_value, value));
	}

	function click_to_natural_position(event: MouseEvent) {
		if (!image_element) return null;

		const image_rectangle = image_element.getBoundingClientRect();
		const display_x = event.clientX - image_rectangle.left;
		const display_y = event.clientY - image_rectangle.top;

		const natural_x = Math.round(display_x * (image_element.naturalWidth / image_rectangle.width));
		const natural_y = Math.round(display_y * (image_element.naturalHeight / image_rectangle.height));

		return {
			x: clamp(natural_x, 0, image_element.naturalWidth - 1),
			y: clamp(natural_y, 0, image_element.naturalHeight - 1)
		};
	}

	function send_request(payload: any, timeout_milliseconds = default_request_timeout_milliseconds): Promise<any> {
		if (!request_topic) return Promise.reject(new Error('ROS not connected'));

		const id = generate_id();
		return new Promise((resolve, reject) => {
			const timeout_handle = setTimeout(() => {
				pending_requests.delete(id);
				reject(new Error(`Timeout waiting for response (${id})`));
			}, timeout_milliseconds);

			pending_requests.set(id, { resolve, timeout_handle });

			request_topic!.publish({
				data: JSON.stringify({ id, ...payload })
			});

		});
	}

	function next_waypoint_name() {
		const waypoint_number = waypoints.length + 1;
		return `WP${String(waypoint_number)}`;
	}

	function add_waypoint_from_response(response: any) {
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

	function delete_waypoint(id: string) {
		waypoints = waypoints.filter((waypoint) => waypoint.id !== id);
	}

	function clear_waypoints() {
		waypoints = [];
	}

	function update_name(id: string, name: string) {
		waypoints = waypoints.map((waypoint) => (waypoint.id === id ? { ...waypoint, name } : waypoint));
	}

	function build_yaml(): string {
		const lines: string[] = [];
		lines.push('waypoints:');

		for (const waypoint of waypoints) {
			lines.push(`- name: ${waypoint.name}`);
			lines.push(`x: ${waypoint.centroid_x / scale}`);
			lines.push(`y: ${waypoint.centroid_y / scale}}`);
			lines.push(`yaw: ${waypoint.yaw}`)
		}

		return lines.join('\n');
	}

	async function copy_yaml() {
		const yaml_text = build_yaml();
		try {
			await navigator.clipboard.writeText(yaml_text);
			status_message = 'YAML copied to clipboard';
		} catch {
			status_message = 'Copy failed (clipboard permission). Select and copy manually.';
		}
	}

	async function on_map_click(event: MouseEvent) {
		const click_position = click_to_natural_position(event);

		current_click_position = click_position;
		contour = [];
		centroid = null;
		sample_image_hexadecimal_color = '';

		if (!click_position) return;

		status_message = `${mode} @ (${click_position.x}, ${click_position.y})…`;

		try {
			const response = await send_request({
				op: 'extract_feature',
				mode, // 'waypoint' | 'border' | 'origin' (protocol value)
				image_id: map_image_id,
				sample_x: click_position.x,
				sample_y: click_position.y,
				tol_h: default_hue_tolerance,
				tol_s: default_saturation_tolerance,
				tol_v: default_value_tolerance
			});

			if (!response?.ok) {
				status_message = response?.message ?? 'Failed';
				return;
			}

			if (mode === 'border') {
				border_contour = Array.isArray(response.contour) ? response.contour : [];
			}

			sample_image_hexadecimal_color = String(response.sample_image_hex ?? '');
			centroid = Array.isArray(response.centroid) ? (response.centroid as [number, number]) : null;
			contour = Array.isArray(response.contour) ? response.contour : [];

			if (mode === 'origin') {
				console.log(`i did it \n ${sample_image_hexadecimal_color}`);
			}

			if (mode === 'waypoint') {
				add_waypoint_from_response(response);
				status_message = centroid
					? `Waypoint added — centroid (${centroid[0]}, ${centroid[1]}) ${sample_image_hexadecimal_color}`
					: `Waypoint added ${sample_image_hexadecimal_color}`;
			} else {
				status_message = `Border OK ${sample_image_hexadecimal_color}`;
			}


		} catch (error: any) {
			status_message = `Error: ${error?.message ?? String(error)}`;
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

	$effect(() => {
		const ros_connection = get_ros_connection();

		if (!ros_connection) {
			response_topic?.unsubscribe();
			request_topic = null;
			response_topic = null;

			pending_requests.forEach((pending_entry) => clearTimeout(pending_entry.timeout_handle));
			pending_requests.clear();
			return;
		}

		request_topic = new ROSLIB.Topic({
			ros: ros_connection,
			name: request_topic_name,
			messageType: 'std_msgs/msg/String'
		});

		response_topic = new ROSLIB.Topic({
			ros: ros_connection,
			name: response_topic_name,
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
			const pending_entry = id ? pending_requests.get(id) : null;
			if (!pending_entry) return;

			clearTimeout(pending_entry.timeout_handle);
			pending_requests.delete(id);
			pending_entry.resolve(response);
		});

		return () => {
			response_topic?.unsubscribe();
			request_topic = null;
			response_topic = null;
		};
	});
</script>
<ScrollArea orientation="vertical" class="relative flex h-full w-full">
	<div class="wrap">
		<div class="topbar">
			<div class="status">Status: {status_message}</div>

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

				<button type="button" onclick={copy_yaml} disabled={waypoints.length === 0}>
					Copy YAML
				</button>
			</div>
		</div>

		<div class="row">
			<div class="frame">
				<img
					bind:this={image_element}
					src={map_image_url}
					width="700"
					alt="map"
					class="map"
					onclick={on_map_click}
					onload={() => {
						if (!image_element) return;
						svg_view_box = `0 0 ${image_element.naturalWidth} ${image_element.naturalHeight}`;
						map_height = image_element.naturalHeight;
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
					<circle cx={waypoint.centroid_x} cy={waypoint.centroid_y} r="6" fill="red" />
				{/each}

				{#each waypoints as waypoint (waypoint.id)}
					<circle cx={waypoint.click_x} cy={waypoint.click_y} r="3.5" fill="yellow" opacity="0.9" />
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
			<table class="tbl" style="width: 30%;">
				<thead>
					<tr>
						<th>Origin X</th>
						<th>Hex</th>
						<th>Origin Y</th>
						<th>Hex</th>
					</tr>
				</thead>
				<tbody>
					<tr>
						<td>
						
						</td>
						<td>
			
						</td>
						<td>
							
						</td>
						<td>

						</td>
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
								<td>
									<input
										class="nameInput"
										value={waypoint.name}
										oninput={(event) =>
											update_name(waypoint.id, (event.target as HTMLInputElement).value)}
									/>
								</td>
								<td>
									<span class="chip" style={`background:${waypoint.hexadecimal_color || '#eee'}`}>
										{waypoint.hexadecimal_color}
									</span>
								</td>
								<td>{(waypoint.centroid_x / scale).toFixed(2)}</td>
								<td>{(waypoint.centroid_y / scale).toFixed(2)}</td>
								<td><input type="number" bind:value={waypoint.yaw}></td>
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
			<textarea class="yamlBox" readonly value={build_yaml()}></textarea>
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
