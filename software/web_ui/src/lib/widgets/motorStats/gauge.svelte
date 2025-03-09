<script lang="ts">
	import { onMount } from 'svelte';

	// source of svg magicness: https://animation-svelte.vercel.app/magic/circular-progress-bar
	let {
		rpm,
		targetRpm,
		current,
		temperature
	}: { rpm: number; targetRpm: number; current: number; temperature: number } = $props();

	onMount(() => {
		rpm = 23;
		targetRpm = 50;
		current = 10;
		temperature = 30;
	});
</script>

<div
	class="relative m-2 overflow-hidden rounded-[50%] border border-card-foreground"
	style:--circle-size="100px"
	style:--transition-length="200ms"
	style="transform: translateZ(0);"
>
	<svg fill="none" class="z-2 size-full" stroke-width="2" viewBox="0 0 100 100">
		<!-- Target RPM -->
		<circle
			r="45"
			stroke="#db26293f"
			style:--stroke-percent={(targetRpm / 2) * 0.8}
			style:--circumference={2 * Math.PI * 45}
		/>
		<circle
			r="45"
			stroke="#db26293f"
			style:--stroke-percent={(-targetRpm / 2) * 0.8}
			style:--circumference={2 * Math.PI * 45}
			style:transform="scale(-1, 1)"
		/>

		<!-- RPM -->
		<circle
			r="45"
			stroke="#db2629"
			style:--stroke-percent={(rpm / 2) * 0.8}
			style:--circumference={2 * Math.PI * 45}
		/>
		<circle
			r="45"
			stroke="#db2629"
			style:--stroke-percent={(-rpm / 2) * 0.8}
			style:--circumference={2 * Math.PI * 45}
			style:transform="scale(-1, 1)"
		/>

		<!-- Amps -->
		<circle
			r="40"
			stroke="#00f"
			style:--stroke-percent={(current / 2) * 0.8}
			style:--circumference={2 * Math.PI * 40}
		/>
		<circle
			r="40"
			stroke="#00f"
			style:--stroke-percent={(-current / 2) * 0.8}
			style:--circumference={2 * Math.PI * 40}
			style:transform="scale(-1, 1)"
		/>
	</svg>
	<div class="absolute inset-0 flex flex-col justify-center">
		<p class="mb-1 ml-[50px] text-card-foreground">Forwards</p>
		<div
			class="w-full border border-dashed border-card-foreground border-t-transparent"
			style:z-index="-1"
		></div>
		<p class="ml-[50px] mt-1 text-card-foreground">Backwards</p>
	</div>
	<!-- rpm overlay -->
	<div
		class="absolute left-[50%] top-[50%] w-[200px] rounded-lg border border-card-foreground bg-background p-2"
		style:transform="translateY(-50%)"
	>
		<p class="text-2xl">RPM: {Math.round(rpm * 100) / 100}</p>
		<p class="">Temp: {Math.round(temperature * 100) / 100}</p>
	</div>
</div>

<style>
	circle {
		cx: 50;
		cy: 50;
		stroke-width: 4;
		stroke-linecap: round;
		stroke-linejoin: round;

		transform: scale(-1, -1);
		stroke-dasharray: calc(var(--stroke-percent) * calc(var(--circumference) / 100))
			var(--circumference);
		transition: all var(--transition-length) ease 0s;
		transform-origin: calc(var(--circle-size) / 2) calc(var(--circle-size) / 2);
	}
</style>
