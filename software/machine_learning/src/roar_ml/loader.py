import os
import yaml
from torch.utils.data import Dataset
import numpy as np

# channel definitions
channels = [
    ("FZ_450"),
    ("FY_555_wide"),
    ("FXL_600"),
    ("NIR_855"),
    ("VIS1"),
    ("F2_425"),
    ("F3_475"),
    ("F4_515"),
    ("F6_640"),
    ("F1_405"),
    ("F7_690"),
    ("F8_745"),
    ("F5_550_narrow"),

    ("FZ_450_2"),
    ("FY_555_wide_2"),
    ("FXL_600_2"),
    ("NIR_855_2"),
    ("VIS1_2"),
    ("F2_425_2"),
    ("F3_475_2"),
    ("F4_515_2"),
    ("F6_640_2"),
    ("F1_405_2"),
    ("F7_690_2"),
    ("F8_745_2"),
    ("F5_550_narrow_2"),



    ("FZ_450_3"),
    ("FY_555_wide_3"),
    ("FXL_600_3"),
    ("NIR_855_3"),
    ("VIS1_3"),
    ("F2_425_3"),
    ("F3_475_3"),
    ("F4_515_3"),
    ("F6_640_3"),
    ("F1_405_3"),
    ("F7_690_3"),
    ("F8_745_3"),
    ("F5_550_narrow_3")

]

EPS = 1e-6

def compute_ratios(raw):
    ch = {name: raw[i] for i, name in enumerate(channels)}
    ratios = [
        ch["F1_405"]/(ch["F5_550_narrow"] + EPS),
        ch["F2_425"] / (ch["F7_690"] + EPS),
        ch["F4_515"] / (ch["F6_640"] + EPS),
        ch["F2_425"] / (ch["F6_640"] + EPS),
        ch["F2_425"] / (ch["F5_550_narrow"] + EPS),
        ch["F5_550_narrow"] / (ch["F7_690"] + EPS),
        ch["F3_475"] / (ch["F7_690"] + EPS),

        ch["F1_405_2"]/(ch["F5_550_narrow_2"] + EPS),
        ch["F2_425_2"] / (ch["F7_690_2"] + EPS),
        ch["F4_515_2"] / (ch["F6_640_2"] + EPS),
        ch["F2_425_2"] / (ch["F6_640_2"] + EPS),
        ch["F2_425_2"] / (ch["F5_550_narrow_2"] + EPS),
        ch["F5_550_narrow_2"] / (ch["F7_690_2"] + EPS),
        ch["F3_475_2"] / (ch["F7_690_2"] + EPS),

        ch["F1_405_3"]/(ch["F5_550_narrow_3"] + EPS),
        ch["F2_425_3"] / (ch["F7_690_3"] + EPS),
        ch["F4_515_3"] / (ch["F6_640_3"] + EPS),
        ch["F2_425_3"] / (ch["F6_640_3"] + EPS),
        ch["F2_425_3"] / (ch["F5_550_narrow_3"] + EPS),
        ch["F5_550_narrow_3"] / (ch["F7_690_3"] + EPS),
        ch["F3_475_3"] / (ch["F7_690_3"] + EPS)
    ]
    return ratios




class IlmeniteDataLoader(Dataset):
    def __init__(self, root_dir, data_file):
        # Getting data from yaml
        data_dir = os.path.join(root_dir, data_file + ".yaml")
        with open(data_dir, "r") as f:
            self.all_data = yaml.safe_load(f)

        

        data = self.all_data["data"]
        labels = self.all_data["labels"]

        self.labeled_data = dict()

        temp = {}
        for key in data.keys():
            if key in labels:
                raw_values = data[key]
                ratios = compute_ratios(raw_values)

                # Making a dict of tuples (label, ratios)
                temp[key] = (labels[key] / 100.0, ratios)

        #scaling/normalising ratios
        all_ratios = np.array([v[1] for v in temp.values()])

        self.scaler_mean  = all_ratios.mean(axis=0)   
        self.scaler_std   = all_ratios.std(axis=0)    
        scaled = (all_ratios - self.scaler_mean) / (self.scaler_std + 1e-8)

        for i, key in enumerate(temp.keys()):
            label = temp[key][0]
            self.labeled_data[key] = (label, scaled[i].tolist())

    def __len__(self):
        return len(self.labeled_data)

    def __getitem__(self, idx):
        
        if idx >= len(self):
            raise IndexError

        #get idx data
        idx_key = list(self.labeled_data.keys())[idx]
        idx_data = self.labeled_data[idx_key]

        
        label = idx_data[0]
        data = idx_data[1]

        return data, label


def main():
    #use absolute path or get from environment/argument
    PATH = "~/perseus-v2/software/machine_learning/data/"
    data_dir = os.environ.get("ROAR_DATA_DIR", os.path.expanduser(PATH))

    IlmeniteDataLoader(data_dir, "test_1").__getitem__(0)
    

if __name__ == "__main__":
    main()
