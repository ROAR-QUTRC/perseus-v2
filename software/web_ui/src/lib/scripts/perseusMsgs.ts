export interface TriggerDeviceRequestType {
  id: number;
  data: number;
  trigger: boolean;
}

export interface TriggerDeviceResponseType {
  success: boolean;
}

export interface RequestInt8ArrayResponseType {
  data: Array<number>;
}
