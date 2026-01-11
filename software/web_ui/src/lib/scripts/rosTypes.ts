export interface ActuatorsMessageType {
  header: object;
  velocity: Array<number>;
  normalized: Array<number>;
}

export interface Float64MultiArrayType {
  data: Array<number>;
}

export interface EmptyRequestType {}

export interface EmptyResponseType {}

export interface SetBoolRequestType {
  data: boolean;
}

export interface SetBoolResponseType {
  success: boolean;
  message: string;
}
