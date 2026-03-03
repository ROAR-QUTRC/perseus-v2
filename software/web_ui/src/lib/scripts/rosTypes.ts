// std_msgs types
export interface Float64MultiArrayType {
  data: Array<number>;
}

// actuator_msgs types
export interface ActuatorsMessageType {
  header: {
<<<<<<< HEAD
    stamp: object; // filled by rosbridge
    frame_id: string;
=======
    stamp?: object; // filled by rosbridge
    frame_id?: string;
>>>>>>> origin/main
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
