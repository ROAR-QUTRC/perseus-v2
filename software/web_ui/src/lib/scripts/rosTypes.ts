interface HeaderType {
  stamp: object; // filled by rosbridge
  frame_id: string;
}

// std_msgs types
export interface Float64MultiArrayType {
  data: Array<number>;
}

export interface TwistMessageType {
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
}

export interface StampedTwistMessageType {
  header: HeaderType;
  twist: TwistMessageType;
}

// actuator_msgs types
export interface ActuatorsMessageType {
  header: {
    stamp: object; // filled by rosbridge
    frame_id: string;
  };
  position: Array<number>; // float64
  velocity: Array<number>; // float64
  normalized: Array<number>; // float64
}

// Service types
export interface EmptyRequestType { }

export interface EmptyResponseType { }

export interface SetBoolRequestType {
  data: boolean;
}

export interface SetBoolResponseType {
  success: boolean;
  message: string;
}

