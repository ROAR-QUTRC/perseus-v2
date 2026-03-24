import os
import torch
import torch.nn as nn
import torch.optim as optim
import matplotlib.pyplot as plt
from torch.utils.data import DataLoader, random_split

from model import IlmeniteModel
from loader import IlmeniteDataLoader

torch.manual_seed(0)

###############################################################################
####   SETTING UP THE DATASET                                              ####
###############################################################################

PATH = "~/projects/perseus-v2/software/machine_learning/data/"
data_dir = os.environ.get("ROAR_DATA_DIR", os.path.expanduser(PATH))

#checking no data is lost
dataset = IlmeniteDataLoader(data_dir, "test_1")
print(f"Dataset contains {len(dataset)} samples")

#split 60% train, 40% validation
train_size = int(len(dataset) * 0.6)
val_size = len(dataset) - train_size
train_dataset, val_dataset = random_split(dataset, [train_size, val_size])
print(f"Train: {train_size} samples | Val: {val_size} samples")

#shuffle = true ensures that the order is randomised for each epoch
train_loader = DataLoader(train_dataset, batch_size=8, shuffle=True) #consider changing batch size to 18
val_loader   = DataLoader(val_dataset,   batch_size=8, shuffle=False) 


###############################################################################
####   INITIALISE THE NETWORK                                              ####
###############################################################################

torch.set_num_threads(7)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")

#creating an instance of th model and moving the weights to the device
net = IlmeniteModel().to(device)

###############################################################################
####   INITIALISE LOSS FUNCTION AND OPTIMISER                             ####
###############################################################################

# MSELoss for regression - predicting continuous concentration values
criterion = nn.MSELoss()
# Adam optimiser (adjusts the learning rate)
optimizer = optim.Adam(net.parameters(), lr=0.001)

#reduce learning rate if val loss stops improving for 5 epochs
scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
    optimizer, mode="min", patience=5, factor=0.5, verbose=True
)

###############################################################################
####   TRAINING LOOP                                                       ####
###############################################################################

losses = {"train": [], "val": []}
best_val_loss = float("inf")

for epoch in range(50):

    # --- Training ---
    net.train()
    epoch_loss = 0.0
    for data, labels in train_loader:
        #converting yaml to float tensors
        data   = torch.tensor(data,   dtype=torch.float32).to(device)
        labels = torch.tensor(labels, dtype=torch.float32).unsqueeze(1).to(device)

        #zero the parameter gradients
        optimizer.zero_grad()
        outputs = net(data)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step() #updating weights via adam optimiser

        #print statistics
        epoch_loss += loss.item()

    train_loss = epoch_loss / len(train_loader)
    losses["train"].append(train_loss)

    #validation stage
    net.eval() #turning droupouts off
    val_loss = 0.0
    with torch.no_grad():
        for data, labels in val_loader:
            data   = torch.tensor(data,   dtype=torch.float32).to(device)
            labels = torch.tensor(labels, dtype=torch.float32).unsqueeze(1).to(device)
            outputs = net(data)
            val_loss += criterion(outputs, labels).item()

    val_loss = val_loss / len(val_loader)
    losses["val"].append(val_loss)

    scheduler.step(val_loss)
    print(f"Epoch {epoch+1:>3}/50 | Train Loss: {train_loss:.6f} | Val Loss: {val_loss:.6f}")

    # Overfitting warning
    if train_loss < val_loss * 0.5:
        print(" Warning: possible overfitting (train loss much lower than val loss)")

    # Save best model
    if val_loss < best_val_loss:
        best_val_loss = val_loss
        torch.save(net.state_dict(), "ilmenite_model.pth")
        print(f" Saved best model (val loss: {best_val_loss:.6f})")

print(f"\nFinished Training. Best val loss: {best_val_loss:.6f}")

########################
## Validation dataset ##
########################



###############################################################################
####   PLOT LOSSES                                                         ####
###############################################################################

plt.plot(losses["train"], label="Training")
plt.plot(losses["val"],   label="Validation")
plt.xlabel("Epoch")
plt.ylabel("Loss")
plt.title("Ilmenite Model - Training vs Validation Loss")
plt.legend()
plt.show()