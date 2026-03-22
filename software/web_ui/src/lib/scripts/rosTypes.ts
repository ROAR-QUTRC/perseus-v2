// std_msgs types
export interface NumberArrayType {
  data: Array<number>;
}

// actuator_msgs types
export interface ActuatorsMessageType {
  header: {
    stamp?: object; // filled by rosbridge
    frame_id?: string;
  };
  position: Array<number>; // float64
  velocity: Array<number>; // float64
  normalized: Array<number>; // float64
}

// Service types
export interface EmptyRequestType {}

export interface EmptyResponseType {}

export interface SetBoolRequestType {
  data: boolean;
}

export interface SetBoolResponseType {
  success: boolean;
  message: string;
}
