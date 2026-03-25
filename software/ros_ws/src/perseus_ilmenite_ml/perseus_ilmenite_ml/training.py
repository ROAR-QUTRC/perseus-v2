import os
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, random_split
import json
from tqdm import tqdm

from model import IlmeniteModel
from loader import IlmeniteDataLoader


torch.manual_seed(0)


#setting up the dataset
PATH = "~/perseus-v2/software/machine_learning/data/"
data_dir = os.environ.get("ROAR_DATA_DIR", os.path.expanduser(PATH))

#checking no data is lost
dataset = IlmeniteDataLoader(data_dir, "test_1")
print(f"Dataset contains {len(dataset)} samples")

#split 80% train, 20% validation
split = 0.8
train_size = int(len(dataset) * split)
val_size = len(dataset) - train_size
train_dataset, val_dataset = random_split(dataset, [train_size, val_size])
print(f"Train: {train_size} samples | Val: {val_size} samples")

#shuffle = true ensures that the order is randomised for each epoch
def collate_fn(batch):
    data   = torch.tensor([item[0] for item in batch], dtype=torch.float32)
    labels = torch.tensor([item[1] for item in batch], dtype=torch.float32)
    return data, labels

train_loader = DataLoader(train_dataset, batch_size=8, shuffle=True,  collate_fn=collate_fn) 
val_loader   = DataLoader(val_dataset,   batch_size=8, shuffle=False, collate_fn=collate_fn)
#initialing the network

torch.set_num_threads(7)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")

net = IlmeniteModel().to(device)

#initialising loss function and optimiser

#MSELoss for regression - predicting continuous concentration values
criterion = nn.MSELoss()
#adam optimiser (adjusts the learning rate)
optimiser = optim.Adam(net.parameters(), lr=0.01)

#reduce learning rate if val loss stops improving for 5 epochs
scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
     optimiser, mode="min", patience=5, factor=0.5
 )


#training loop
losses = {"train": [], "val": []}
best_val_loss = float("inf")
# train_loss_arr = []

NUM_OF_EPOCHS = 100

for epoch in range(NUM_OF_EPOCHS):

    net.train()
    epoch_loss = 0.0
    for data, labels in tqdm(train_loader):
        #converting yaml to float tensors
        data   = data.to(device)
        # print(len(data))
        labels = labels.unsqueeze(1).to(device)

        #zero the parameter gradients
        optimiser.zero_grad()
        outputs = net(data)
        loss = criterion(outputs, labels)
        loss.backward()
        optimiser.step() #updating weights via adam optimiser

        epoch_loss += loss.item()

        train_loss = epoch_loss / len(train_loader)
    losses["train"].append(train_loss)

    #validation
    net.eval() #turning dropouts off
    val_loss = 0.0
    with torch.no_grad(): #optimiser is not needed cuz weights have been finalised
        for data, labels in val_loader:
            data   = data.to(device)
            labels = labels.unsqueeze(1).to(device)
            outputs = net(data)
            val_loss += criterion(outputs, labels).item()

    val_loss = val_loss / len(val_loader)
    losses["val"].append(val_loss)

    scheduler.step(val_loss)
    print(f"Epoch {epoch+1:>3}/{NUM_OF_EPOCHS} | Train Loss: {train_loss:.6f} | Val Loss: {val_loss:.6f}")

    #overfitting warning
    if train_loss < val_loss * 0.5:
        print("Warning: possible overfitting")

    #save best model
    if val_loss < best_val_loss:
        best_val_loss = val_loss
        torch.save(net.state_dict(), "ilmenite_model.pth")
        print(f"Saved best model (val loss: {best_val_loss:.6f})")


for loss in losses["train"]:
    print(f"{loss}")

print("printing validation losses")

for loss in losses["val"]:
     print(f"{loss:.6f}")

print(f"\nFinished Training. Best val loss: {best_val_loss:.6f}")

#Save the scaler so deploy.py can apply identical scaling to new sensor data
scaler_data = {
    "mean": dataset.scaler_mean.tolist(),
    "std":  dataset.scaler_std.tolist()
}
with open("ilmenite_scaler.json", "w") as f:
    json.dump(scaler_data, f)
print("Scaler saved to: ilmenite_scaler.json")