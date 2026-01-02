export interface ActuatorsMessage {
	header: object;
	velocity: Array<number>;
	normalized: Array<number>;
}

// Generic stamped message definition
// This is used for internal typing only
interface Stamped {
	header: object;
}

export interface TwistStampedMessage extends Stamped {
	twist: {
		linear: {
			x: number;
			y: number;
			z: number;
		};
		angular: {
			x: number;
			y: number;
			z: number;
		};
	};
}
