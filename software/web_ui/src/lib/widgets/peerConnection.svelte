<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Peer connection';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	import { onMount } from 'svelte';

	let localVideo = $state<null | HTMLVideoElement>(null);
	let remoteVideo = $state<null | HTMLVideoElement>(null);

	let localStream: MediaStream | null = null;
	let startTime;

	let pc1: RTCPeerConnection | null = null;
	let pc2: RTCPeerConnection | null = null;

	const start = async () => {
		try {
			const stream = await navigator.mediaDevices.getUserMedia({ video: true, audio: false });
			localVideo!.srcObject = stream;
			localStream = stream;
		} catch (e: any) {
			alert(`getUserMedia() error: ${e.name}`);
		}
	};

	const call = async () => {
		startTime = window.performance.now();
		const videoTracks = localStream!.getVideoTracks();
		if (videoTracks.length > 0) {
			console.log(`Using video device: ${videoTracks[0].label}`);
		}

		const configuration = { iceServers: [] };
		console.log('RTCPeerConnection configuration:', configuration);

		pc1 = new RTCPeerConnection(configuration);
		console.log('Created local peer connection object pc1');
		pc1.addEventListener('icecandidate', async (e) => {
			try {
				await pc2?.addIceCandidate(e.candidate!);
			} catch (e: any) {
				console.error('Error adding received ice candidate', e);
			}
			console.log(`pc1 ICE candidate:\n${e.candidate ? e.candidate.candidate : '(null)'}`);
		});
		pc2 = new RTCPeerConnection(configuration);
		console.log('Created local peer connection object pc2');
		pc2.addEventListener('icecandidate', async (e) => {
			try {
				await pc1?.addIceCandidate(e.candidate!);
			} catch (e: any) {
				console.error('Error adding received ice candidate', e);
			}
			console.log(`pc2 ICE candidate:\n${e.candidate ? e.candidate.candidate : '(null)'}`);
		});
		pc1.addEventListener('iceconnectionstatechange', (e) => {
			if (pc1) {
				console.log(`pc1 ICE state: ${pc1!.iceConnectionState}`);
				console.log('ICE state change event: ', e);
			}
		});
		pc2.addEventListener('iceconnectionstatechange', (e) => {
			if (pc2) {
				console.log(`pc1 ICE state: ${pc2!.iceConnectionState}`);
				console.log('ICE state change event: ', e);
			}
		});

		pc2.addEventListener('track', (e) => {
			if (remoteVideo?.srcObject !== e.streams[0]) {
				remoteVideo!.srcObject = e.streams[0];
				console.log('pc2 received remote stream');
			}
		});

		localStream!.getTracks().forEach((track) => {
			pc1!.addTrack(track, localStream!);
		});

		try {
			const offer = await pc1.createOffer({
				offerToReceiveAudio: false,
				offerToReceiveVideo: true
			});
			console.log('offer from pc1:', offer.sdp);
			try {
				await pc1.setLocalDescription(offer);
				console.log('pc1 setLocalDescription complete');
			} catch (error: any) {
				console.log(`Failed to set session description: ${error.toString()}`);
			}

			try {
				await pc2.setRemoteDescription(offer);
				console.log('pc2 setRemoteDescription complete');
			} catch (error: any) {
				console.log(`Failed to set session description: ${error.toString()}`);
			}

			try {
				const answer = await pc2.createAnswer();
				try {
					await pc2.setLocalDescription(answer);
				} catch (error: any) {
					console.log(`Failed to set session description: ${error.toString()}`);
				}

				try {
					await pc1.setRemoteDescription(answer);
				} catch (error: any) {
					console.log(`Failed to set session description: ${error.toString()}`);
				}
			} catch (error: any) {
				console.log(`Failed to create session description: ${error.toString()}`);
			}
		} catch (error: any) {
			console.log(`Failed to create session description: ${error.toString()}`);
		}
	};

	const hangup = () => {
		console.log('Ending call');
		pc1!.close();
		pc2!.close();
		pc1 = null;
		pc2 = null;
	};
</script>

<div class="flex">
	<video bind:this={localVideo} playsinline autoplay muted></video>
	<!-- svelte-ignore a11y_media_has_caption -->
	<video bind:this={remoteVideo} playsinline autoplay></video>
</div>

<div class="box">
	<button onclick={start}>Start</button>
	<button onclick={call}>Call</button>
	<button onclick={hangup}>Hang Up</button>
</div>
