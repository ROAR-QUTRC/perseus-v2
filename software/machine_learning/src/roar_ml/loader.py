import os
import yaml
from torch.utils.data import Dataset


class IlmeniteDataLoader(Dataset):
    def __init__(self, root_dir, data_file):

        # Getting data from yaml
        data_dir = os.path.join(root_dir, data_file + ".yaml")
        with open(data_dir, "r") as f:
            self.all_data = yaml.safe_load(f)

        data = self.all_data["data"]
        labels = self.all_data["labels"]

        self.labeled_data = dict()

        for key in data.keys():
            if key in labels:
                # Making a dict of tuples (label, data)
                self.labeled_data[key] = (labels[key], data[key])

    def __len__(self):
        return len(self.labeled_data)

    def __getitem__(self, idx):

        # Reject out of range
        if idx >= len(self):
            raise IndexError

        # Get idx data
        idx_key = list(self.labeled_data.keys())[idx]
        idx_data = self.labeled_data[idx_key]

        # Extract out values
        label = idx_data[0]
        data = idx_data[1]

        return data, label


def main():
    # Use absolute path or get from environment/argument
    PATH = "~/projects/perseus-v2/software/machine_learning/data/"
    data_dir = os.environ.get(
        "ROAR_DATA_DIR",
        os.path.expanduser(PATH)
    )

    IlmeniteDataLoader(data_dir, "test_1").__getitem__(0)


if __name__ == "__main__":
    main()
