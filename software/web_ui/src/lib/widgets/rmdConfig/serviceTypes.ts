export interface RMDStatusRequest {
	motor_id: number;
}

export interface RMDStatusResponse {
	motor_id: number;
	temperature: number;
	brake_control: boolean;
	voltage: number;
	torque_current: number;
	speed: number;
	angle: number;
	phase_a_current: number;
	phase_b_current: number;
	phase_c_current: number;
	error: number;
}

export interface RmdCanIdResponse {
	servo_ids: number[];
}

export interface RmdBrakeRequest {
	motor_id: number;
	brake_enable?: boolean;
}

export interface RmdDataRequest {
	motor_id: number;
	trigger?: boolean;
	data?: number;
}

export interface SuccessServiceResponse {
	success: boolean;
}
