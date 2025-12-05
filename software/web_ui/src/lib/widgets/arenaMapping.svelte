<script lang="ts" module>
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';
	import { isNullOrUndef } from 'chart.js/helpers';

	export const name = 'Arena Map Editor';
	export const description = 'Create the YAML file that Perseus will use to navigate';
	export const group = 'Mapping';
	export const isRosDependent = true;

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/ros-bridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import ROSLIB from 'roslib';
	type Point = { x: number; y: number };

	let points = $state([] as Point[]);
	let originIndex = $state<number | null>(null);

	let background_image = $state<string | null>(null);
	let naturalWidth = $state(1);
	let naturalHeight = $state(1);
	let displayWidth = $state(300);
	let displayHeight = $state(300);
	let scale = $state(1);
	let val_X = $state(1);
	let val_Y = $state(1);
	let eyeDropperSupported = typeof window !== 'undefined' && 'EyeDropper' in window;
	const colourArea = ['Point', 'Border', 'Grid'];
	let pickedColor = $state<(string | null)[]>(colourArea.map(() => null));

	async function pickColour(index: number) {
		if (!eyeDropperSupported) {
			console.warn('EyeDropper API not supported in this browser');
			return;
		}
		try {
			const ed = new (window as any).EyeDropper();
			const { sRGBHex } = await ed.open(); // e.g. "#a1b2c3"
			pickedColor[index] = sRGBHex;
			// here you can also store it on a point, etc.
		} catch (err) {
			// user pressed Esc or cancelled
			console.warn('EyeDropper cancelled or failed', err);
		}
	}
	// uploads the image to the graph
	function handle_upload(event: Event) {
		const input = event.target as HTMLInputElement;
		const file = input.files?.[0];
		if (!file) return;

		const url = URL.createObjectURL(file);
		const img = new Image();
		//gets the height and width of the uploaded image
		img.onload = () => {
			naturalWidth = img.width;
			naturalHeight = img.height;

			// scale so height <= 500px,
			scale = 500 / naturalHeight;
			if (scale > 1) scale = 1;

			displayWidth = naturalWidth * scale;
			displayHeight = naturalHeight * scale;
			background_image = url;
		};

		img.src = url;
	}
	//clears the background image
	function clear_image() {
		background_image = null;
		points = [];
		originIndex = null;
		naturalWidth = 1;
		naturalHeight = 1;
		displayWidth = 500;
		displayHeight = 500;
		scale = 1;
	}

	//add the point to the uploaded image
	function add_point(event: MouseEvent) {
		// Require an image first so scaling is valid
		if (!background_image) return;

		const target = event.currentTarget as HTMLElement | null;
		if (!target) return;

		const rect = target.getBoundingClientRect();
		const xDisplay = event.clientX - rect.left;
		const yDisplay = event.clientY - rect.top;

		const x = xDisplay / scale;
		const y = yDisplay / scale;

		points = [...points, { x, y }];
	}

	// handle keyboard
	function handle_graph_keydown(event: KeyboardEvent) {
		if (event.key === 'Enter' || event.key === ' ') {
			event.preventDefault();
		}
	}
	//sets the origin the other points will go off
	function set_origin(index: number) {
		originIndex = index;
	}

	//deletes a point
	function delete_point(index: number) {
		points = points.filter((_, i) => i !== index);

		if (originIndex !== null) {
			if (index === originIndex) {
				originIndex = null;
			} else if (index < originIndex) {
				originIndex = originIndex - 1;
			}
		}
	}
	//finds if there is a specific origin point or the origin is the default
	function get_origin_point(): Point | null {
		if (originIndex === null) return null;
		return points[originIndex] ?? null;
	}
	//creates the custom origin point based off a point on the graph
	function custom_origin(p: Point): Point {
		const origin = get_origin_point();
		if (!origin) return { x: p.x, y: p.y };
		return {
			x: (p.x - origin.x) * -val_X, //* all this by -ve to swap axis, create direction vairables
			y: (p.y - origin.y) * -val_Y
		};
	}
	let listener: ROSLIB.Topic | null = null;
	listener = new ROSLIB.Topic({
				ros: getRosConnection() as ROSLIB.Ros,
				name: '/test_publisher',
				messageType: 'std_msgs/String'
			});
			//
// yarn dev, yarn start
			listener.subscribe((message: any) => {
				//const { name, status, current, voltage } = JSON.parse(message.data);
				// busState[name] = {
				// 	status: Number(status),
				// 	current: Number(current),
				// 	voltage: Number(voltage),
				// 	openAlert: busState[name]?.openAlert ?? false
			//	};
			});
	const publisher = new ROSLIB.Topic({
		ros: getRosConnection() as ROSLIB.Ros,
		name: '/test_subscriber', // change this
		messageType: 'std_msgs/Int16MultiArray'
	});
	// const message = new ROSLIB.Message({
	// 	data: JSON.stringify({
	// 		bus: bus,
	// 		on: busState[bus].status !== 1 ? '1' : '0',
	// 		clear: busState[bus].status === 6 || busState[bus].status === 4 ? '1' : '0' // Clear FAULT or SWITCH_FAILED
	// 	})
	// });
</script>

<div class="widget-shell">
	<div class="widget-scroll">
		<h1>Arena Map</h1>
		<h4>Upload image of arena</h4>

		<input type="file" accept="image/*" onchange={handle_upload} />

		<button type="button" class="btn" onclick={clear_image} disabled={!background_image}
			>Clear image
		</button>
		<div class="layout">
			<div class="graph-column">
				<div
					class="graph"
					role="button"
					aria-label="Arena map editor; click to add points"
					tabindex="0"
					onclick={add_point}
					onkeydown={handle_graph_keydown}
					style="
        width: {displayWidth}px;
        height: {displayHeight}px;
        background-image: url({background_image});
      "
				>
					{#each points as p, i}
						<div
							class="point {originIndex === i ? 'origin-point' : ''}"
							style="
            left: {p.x * scale}px;
            top: {p.y * scale}px;
          "
							title={originIndex === i ? 'Origin' : `Point ${i + 1}`}
						></div>
					{/each}
				</div>
			</div>

			<aside class="side-column">
				<h4>Eyedropper</h4>
				{#each colourArea as label, i}
					<div class="mb-2 flex items-center gap-2">
						<button
							type="button"
							class="btn"
							onclick={() => pickColour(i)}
							disabled={!eyeDropperSupported}
						>
							{eyeDropperSupported ? `Pick colour for ${label}` : 'Not supported'}
						</button>

						{#if pickedColor[i]}
							<div
								class="h-4 w-4 rounded border border-slate-400"
								style={`background-color: ${pickedColor[i]};`}
							></div>
							<span class="font-mono text-[10px]">{pickedColor[i]}</span>
						{/if}
					</div>
				{/each}
				{#if pickedColor}
					<div class="eyedropper-result">
						<div class="color-swatch" style={'background-color: ' + pickedColor}></div>
						<span>{pickedColor}</span>
					</div>
				{/if}
			</aside>
		</div>

		<div class="table-container">
			<table>
				<thead>
					<tr>
						<th>#</th>
						<th>Rel Y</th>
						<th>Rel X</th>
						<th>Abs X</th>
						<th>Abs Y</th>
						<th>Origin</th>
						<th>Delete</th>
					</tr>
				</thead>
				<tbody>
					{#each points as row, i}
						{@const c = custom_origin(row)}
						<tr>
							<td>{i + 1}</td>

							<!-- Relative (read-only) -->
							<td>
								<input class="coord-input" value={c.x.toFixed(1)} readonly />
							</td>
							<td>
								<input class="coord-input" value={c.y.toFixed(1)} readonly />
							</td>

							<!-- Absolute (editable) -->
							<td>
								<input class="coord-input" type="number" step="0.1" bind:value={row.x} />
							</td>
							<td>
								<input class="coord-input" type="number" step="0.1" bind:value={row.y} />
							</td>
							<td>
								<button type="button" class="btn" onclick={() => set_origin(i)}>
									{originIndex === i ? 'Origin' : 'Use as origin'}
								</button>
							</td>

							<td>
								<button type="button" class="btn" onclick={() => delete_point(i)}> Delete </button>
							</td>
						</tr>
					{/each}
				</tbody>
			</table>
		</div>
	</div>
</div>

<style>
	.graph {
		border: 2px solid #333;
		position: relative;
		background-size: contain;
		background-repeat: no-repeat;
		background-position: center;
	}

	.point {
		width: 8px;
		height: 8px;
		background-color: red;
		border-radius: 50%;
		position: absolute;
		transform: translate(-50%, -50%);
	}

	.origin-point {
		background-color: #00c853;
	}

	.table-container {
		/* max-height: 260px;  <-- removed */
		margin-top: 20px;
	}

	table {
		width: 100%;
		border-collapse: collapse;
	}

	th,
	td {
		border: 1px solid #ccc;
		padding: 4px 6px;
		font-size: 0.85rem;
	}

	.coord-input {
		width: 70px;
		padding: 2px 4px;
		font-size: 0.8rem;
	}

	.btn {
		padding: 2px 6px;
		font-size: 0.75rem;
		cursor: pointer;
	}

	.layout {
		display: flex;
		gap: 6px;
		margin-top: 12px;
		align-items: flex-start;
		min-width: 0;
	}

	.graph-column {
		flex: 1 1 auto;
		min-width: 0;
	}

	.side-column {
		flex: 0 0 220px;
		max-height: 100%;
		overflow-y: auto;
		border: 1px solid #ccc;
		border-radius: 4px;
		padding: 8px;
		background: #000000;
		font-size: 0.85rem;
	}

	.eyedropper-result {
		display: flex;
		align-items: center;
		gap: 8px;
		margin-top: 8px;
	}

	.color-swatch {
		width: 20px;
		height: 20px;
		border: 1px solid #999;
		border-radius: 2px;
	}

	/* Widget wrapper + scroll behaviour */
	.widget-shell {
		height: 100%; /* or 100vh if needed */
		display: flex;
		flex-direction: column;
	}

	.widget-scroll {
		flex: 1 1 auto;
		min-height: 0;
		overflow-y: auto;
		padding: 8px;
	}
</style>
