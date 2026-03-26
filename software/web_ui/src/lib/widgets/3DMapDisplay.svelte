<script lang="ts" module>
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';
	export const name = '3D Map Viewer';
	export const description = 'Displays a 3D map.';
	export const group: WidgetGroupType = 'Misc';
	export const isRosDependent = false;
	export const settings: WidgetSettingsType = $state({ groups: {} });
</script>

<script lang="ts">
	import { onMount, onDestroy } from 'svelte';
	import * as THREE from 'three';
	import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';

	let container: HTMLDivElement;
	let renderer: THREE.WebGLRenderer;
	let label = $state('Loading...');
	let indexDisplay = $state('');

	onMount(() => {
		const scene = new THREE.Scene();
		scene.background = new THREE.Color('#1a1a2e');
		const camera = new THREE.PerspectiveCamera(60, container.clientWidth / container.clientHeight, 0.1, 1000);

		renderer = new THREE.WebGLRenderer({ antialias: true });
		renderer.setSize(container.clientWidth, container.clientHeight);
		renderer.domElement.style.pointerEvents = 'auto';
		container.appendChild(renderer.domElement);

		scene.add(new THREE.AmbientLight(0xffffff, 0.6));
		const dir = new THREE.DirectionalLight(0xffffff, 1.2);
		dir.position.set(10, 20, 10);
		scene.add(dir);

		let center = new THREE.Vector3();
		let maxDim = 10;
		let theta = -0.115;
		let phi = 2.7;
		let radius = 1;
		let currentMesh: THREE.Group | null = null;
		let currentFile = '';

		const updateCamera = () => {
			const r = maxDim * radius;
			camera.position.set(
				center.x + r * Math.sin(phi) * Math.sin(theta),
				center.y + r * Math.cos(phi),
				center.z + r * Math.sin(phi) * Math.cos(theta)
			);
			camera.lookAt(center);
			renderer.render(scene, camera);
		};

		const loadFile = (path: string) => {
			if (currentMesh) {
				scene.remove(currentMesh);
				currentMesh = null;
			}
			label = path.split('/').pop() ?? path;
			new GLTFLoader().load(path, (gltf) => {
				gltf.scene.traverse((obj) => {
					if ((obj as THREE.Mesh).isMesh) {
    					(obj as THREE.Mesh).material = new THREE.MeshStandardMaterial({
        				vertexColors: true,
        				side: THREE.DoubleSide,
    })
}
				});
				currentMesh = gltf.scene;
				scene.add(currentMesh);
				const box = new THREE.Box3().setFromObject(currentMesh);
				box.getCenter(center);
				const size = box.getSize(new THREE.Vector3());
				maxDim = Math.max(size.x, size.y, size.z);
				updateCamera();
			});
		};

		const loadLatest = async () => {
			const res = await fetch('/api/maps');
			const list: string[] = await res.json();
			if (list.length === 0) return;
			const latest = list[list.length - 1];
			if (latest === currentFile) return;
			currentFile = latest;
			indexDisplay = `Latest: ${latest}`;
			loadFile(`/maps/${latest}`);
		};

		loadLatest();
		const interval = setInterval(loadLatest, 5000);

		let dragging = false;
		let lastX = 0;
		let lastY = 0;

		renderer.domElement.addEventListener('mousedown', (e) => { dragging = true; lastX = e.clientX; lastY = e.clientY; e.stopPropagation(); });
		renderer.domElement.addEventListener('mousemove', (e) => {
			if (!dragging) return;
			theta -= (e.clientX - lastX) * 0.005;
			phi = Math.max(0.1, Math.min(Math.PI - 0.1, phi + (e.clientY - lastY) * 0.005));
			lastX = e.clientX; lastY = e.clientY;
			updateCamera();
			e.stopPropagation();
		});
		renderer.domElement.addEventListener('mouseup', (e) => { dragging = false; e.stopPropagation(); });
		renderer.domElement.addEventListener('mouseleave', () => { dragging = false; });
		renderer.domElement.addEventListener('wheel', (e) => {
			radius = Math.max(0.5, Math.min(10, radius + e.deltaY * 0.001));
			updateCamera();
			e.stopPropagation();
			e.preventDefault();
		}, { passive: false });

		const resizeObserver = new ResizeObserver(() => {
			camera.aspect = container.clientWidth / container.clientHeight;
			camera.updateProjectionMatrix();
			renderer.setSize(container.clientWidth, container.clientHeight);
			updateCamera();
		});
		resizeObserver.observe(container);

		return () => {
			clearInterval(interval);
			resizeObserver.disconnect();
		};
	});

	onDestroy(() => renderer?.dispose());
</script>

<div class="relative h-full w-full overflow-hidden">
	<div bind:this={container} class="h-full w-full"></div>
	<div class="absolute bottom-2 left-0 right-0 flex items-center justify-center gap-3 pointer-events-none">
		{#if label.startsWith('scan_')}
			{@const parts = label.replace('scan_', '').replace('.glb', '').split('_')}
			{@const date = parts[0]}
			{@const time = parts[1]}
			{@const formatted = `${date.slice(0,4)}-${date.slice(4,6)}-${date.slice(6,8)} ${time.slice(0,2)}:${time.slice(2,4)}:${time.slice(4,6)}`}
			<span class="rounded bg-black/50 px-2 py-1 text-xs text-white">Current Map Data: {formatted}</span>
		{:else}
			<span class="rounded bg-black/50 px-2 py-1 text-xs text-white">{label}</span>
		{/if}
	</div>
</div>