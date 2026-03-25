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
    return model

def load_scaler(scaler_path):
    with open(scaler_path, "r") as f:
        scaler = json.load(f)
        scaler["mean"] = np.array(scaler["mean"])
        scaler["std"]  = np.array(scaler["std"])
        return scaler

#input is an array
def get_sensor_reading(input_array):
    extracted_array = [float(x.strip()) for x in input_array.split(',')]
    extracted_array_13 = extracted_array.pop(12)
    return extracted_array

#def get_sensor_reading():
    #raw_values = input("type in 39 values with a comma in between")
    #extracted_array = [float(x.strip()) for x in raw_values.split(',')]
    #return extracted_array

def snap_to_nearest(value):
    levels = [0.000, 0.020, 0.050, 0.075, 0.100, 0.125, 0.150, 0.175, 1.000]
    return min(levels, key=lambda x: abs(x - value))


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
            sensor_cols = ",".join([f"reading_{i+1}" for i in range(len(raw_sample))])
            f.write(f"{sensor_cols},predicted_concentration\n")
            

        sensor_vals = ",".join(str(v) for v in raw_sample)
        f.write(f"{sensor_vals},{concentration:.6f}\n")


def main():
    #setting up network
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    model = load_model(WEIGHTS_PATH, device)
    scaler = load_scaler(SCALER_PATH)

    print(f"Logging predictions to: {LOG_PATH}")

    try:
        while True:
            raw_sample = get_sensor_reading()

            if len(raw_sample) != 39:
                print(f"Warning: expected 39 sensor values, got {len(raw_sample)} — skipping")
                time.sleep(POLL_INTERVAL)
                continue

            #run through model
            concentration = predict_concentration(model, raw_sample, scaler, device)
            rounded_concentration = snap_to_nearest(concentration)

            log_prediction(LOG_PATH, raw_sample, concentration)

            print(f"Concentration: {concentration:.4f} | Rounded Concentration: {rounded_concentration:.2%}")
            time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    main()