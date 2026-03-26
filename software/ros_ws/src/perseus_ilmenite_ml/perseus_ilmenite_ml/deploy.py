import os
import time
import torch
import json
import numpy as np
import datetime

from perseus_ilmenite_ml.model import IlmeniteModel
from perseus_ilmenite_ml.loader import compute_ratios
from rosidl_runtime_py import message_to_ordereddict


#configuration
WEIGHTS_PATH = "ilmenite_model.pth"
LOG_PATH = "predictions_log.csv"
POLL_INTERVAL = 1.0 
SCALER_PATH = "ilmenite_scaler.json"


def load_model(weights_path, device):
    model = IlmeniteModel().to(device)
    model.load_state_dict(torch.load(weights_path, map_location=device))
    model.eval()
    print(f"Model loaded from:{weights_path}")
    return model


def load_scaler(scaler_path):
    with open(scaler_path, "r") as f:
        scaler = json.load(f)
    scaler["mean"] = np.array(scaler["mean"])
    scaler["std"]  = np.array(scaler["std"])
    print(f"Scaler loaded from: {scaler_path}")
    return scaler

#input is an array
def get_sensor_reading(input_msg):
    input_array = message_to_ordereddict(input_msg)
    led_settings = ['no_led_reading', 'white_led_reading', 'uv_led_reading']
    samples = ['f2_425nm','fz_450nm', 'f3_475nm', 'f4_515nm', 'f5_550nm', 'fy_555nm', 'fxl_600nm', 'f6_640nm', 'f7_690nm', 'f8_745nm', 'nir_855nm']
    extracted_array = []
    for l in led_settings:
        for s in samples:
            extracted_array.append(f"{input_array[l][s]}")
    print(extracted_array)
    return

#def get_sensor_reading():
    #raw_values = input("Enter 39 channel values separated by commas:\n> ")
    #extracted_array = [float(x.strip()) for x in raw_values.split(',')]
    #return extracted_array


def snap_to_nearest(value):
    levels = [0.000, 0.025, 0.050, 0.075, 0.100, 0.125, 0.150, 0.175]
    return min(levels, key=lambda x: abs(x - value))


def predict_concentration(model, raw_sample, scaler, device):
    features = np.array(compute_features(raw_sample))

    #Scale using the scaler fitted on training data
    features_scaled = (features - scaler["mean"]) / (scaler["std"] + 1e-8)

    tensor = torch.tensor(features_scaled, dtype=torch.float32).unsqueeze(0).to(device)
    with torch.no_grad(): # no optimiser required
        prediction = model(tensor)
    return prediction.item()


def log_prediction(log_path, raw_sample, concentration):
    file_exists = os.path.isfile(log_path) #creating file if it does not exist
    with open(log_path, "a") as f:
        #wrtie header on first run
        if not file_exists:
            channel_cols = ",".join([f"channel_{i+1}" for i in range(len(raw_sample))])
            f.write(f"{channel_cols},predicted_concentration,nearest_level\n")

        timestamp   = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        channel_vals = ",".join(str(v) for v in raw_sample)
        nearest     = snap_to_nearest(concentration)
        f.write(f"{timestamp},{channel_vals},{concentration:.6f},{nearest:.3f}\n")

        sensor_vals = ",".join(str(v) for v in raw_sample)
        f.write(f"{sensor_vals},{concentration:.6f}\n")


def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    model  = load_model(WEIGHTS_PATH, device)
    scaler = load_scaler(SCALER_PATH)

    print(f"Logging to: {LOG_PATH}")

    try:
        while True:
            raw_sample = get_sensor_reading()

            #checking for 39 values
            if len(raw_sample) != 39:
                print(f"Warning: expected 39 values, got {len(raw_sample)} — skipping")
                time.sleep(POLL_INTERVAL)
                continue

            #predicting
            concentration = predict_concentration(model, raw_sample, scaler, device)
            nearest       = snap_to_nearest(concentration)

            #reporting concentration
            log_prediction(LOG_PATH, raw_sample, concentration)
            print(f"Concentration: {concentration:.4f} | Nearest level: {nearest:.2%} | Logged to {LOG_PATH}")

            time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()