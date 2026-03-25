import os
import time
import torch
import json
import numpy as np


from model import IlmeniteModel
from loader import compute_ratios


#configuration
WEIGHTS_PATH = "ilmenite_model.pth"
LOG_PATH = "predictions_log.csv"
POLL_INTERVAL = 1.0 
SCALER_PATH = "ilmenite_scaler.json"


def load_model(weights_path, device):
    model = IlmeniteModel().to(device)
    model.load_state_dict(torch.load(weights_path, map_location=device))
    model.eval()
    print(f"Model loaded from: {weights_path}")
    return model

def load_scaler(scaler_path):
    with open(scaler_path, "r") as f:
        scaler = json.load(f)
        scaler["mean"] = np.array(scaler["mean"])
        scaler["std"]  = np.array(scaler["std"])
        print(f"Scaler loaded from: {scaler_path}")
        return scaler


def get_sensor_reading():
    test_values = input("Enter 18 channel values separated by commas:\n> ")
    input_array = [float(x.strip()) for x in test_values.split(',')]
    return input_array

def predict_concentration(model, raw_sample, scaler, device):
    ratios = np.array(compute_ratios(raw_sample))
    ratios_scaled = (ratios - scaler["mean"]) / (scaler["std"] + 1e-8)
    tensor = torch.tensor(ratios_scaled, dtype=torch.float32).unsqueeze(0).to(device)

    with torch.no_grad(): #no optimiser required
        prediction = model(tensor)
    return prediction.item()


def log_prediction(log_path, raw_sample, concentration):
    file_exists = os.path.isfile(log_path) #creating file if it does not exist
    with open(log_path, "a") as f:
        if not file_exists:
            # Write header on first run
            sensor_cols = ",".join([f"sensor_{i+1}" for i in range(len(raw_sample))])
            f.write(f"{sensor_cols},predicted_concentration\n")

        sensor_vals = ",".join(str(v) for v in raw_sample)
        f.write(f",{sensor_vals},{concentration:.6f}\n")

def snap_to_nearest(value):
    levels = [0.00, 0.02, 0.05, 0.10, 0.15]
    return min(levels, key=lambda x: abs(x - value))

def main():
    #setting up network
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    model = load_model(WEIGHTS_PATH, device)
    scaler = load_scaler(SCALER_PATH)

    print(f"Logging predictions to: {LOG_PATH}")
    print("Running — press Ctrl+C to stop\n")

    try:
        while True:
            raw_sample = get_sensor_reading()

            if len(raw_sample) != 18:
                print(f"Warning: expected 18 sensor values, got {len(raw_sample)} — skipping")
                time.sleep(POLL_INTERVAL)
                continue

            #run through model
            
            concentration = predict_concentration(model, raw_sample, scaler, device)
            rounded_concentration = snap_to_nearest(concentration)

            log_prediction(LOG_PATH, raw_sample, concentration)

            print(f"Concentration: {concentration:.4f} | Rounded Concentration: {rounded_concentration:.2%} | Logged to {LOG_PATH}")
            time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        print("\nStopped.")