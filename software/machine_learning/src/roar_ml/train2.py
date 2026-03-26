import os
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, random_split

from model import IlmeniteModel
from loader import IlmeniteDataLoader


def train(model, loader, optimizer, criterion, device):
    model.train()
    total_loss = 0.0
    for data, labels in loader:
        data = torch.tensor(data, dtype=torch.float32).to(device)
        labels = torch.tensor(labels, dtype=torch.float32).unsqueeze(1).to(device)

        optimizer.zero_grad()
        predictions = model(data)
        loss = criterion(predictions, labels)
        loss.backward()
        optimizer.step()

        total_loss += loss.item()
    return total_loss / len(loader)


def evaluate(model, loader, criterion, device):
    model.eval()
    total_loss = 0.0
    with torch.no_grad():
        for data, labels in loader:
            data = torch.tensor(data, dtype=torch.float32).to(device)
            labels = torch.tensor(labels, dtype=torch.float32).unsqueeze(1).to(device)

            predictions = model(data)
            loss = criterion(predictions, labels)
            total_loss += loss.item()
    return total_loss / len(loader)


def main():
    # --- Config ---
    EPOCHS = 50
    BATCH_SIZE = 16
    LEARNING_RATE = 1e-3
    TRAIN_SPLIT = 0.8
    SAVE_PATH = "ilmenite_model.pth"

    # Set number of threads
    torch.set_num_threads(7)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # --- Data ---
    PATH = "~/projects/perseus-v2/software/machine_learning/data/"
    data_dir = os.environ.get("ROAR_DATA_DIR", os.path.expanduser(PATH))

    dataset = IlmeniteDataLoader(data_dir, "test_1")
    print(f"Dataset size: {len(dataset)} samples")

    train_size = int(len(dataset) * TRAIN_SPLIT)
    val_size = len(dataset) - train_size
    train_dataset, val_dataset = random_split(dataset, [train_size, val_size])
    print(f"Train: {train_size} samples | Val: {val_size} samples")

    train_loader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=BATCH_SIZE, shuffle=False)

    #model, using adam optimiser
    model = IlmeniteModel().to(device)

    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)

    #reduce learning rate if value loss remains the same number for 5 epochs
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode="min", patience=5, factor=0.5, verbose=True
    )

    # training loop 
    best_val_loss = float("inf")
    for epoch in range(1, EPOCHS + 1):
        train_loss = train(model, train_loader, optimizer, criterion, device)
        val_loss = evaluate(model, val_loader, criterion, device)
        scheduler.step(val_loss)

        print(f"Epoch {epoch:>3}/{EPOCHS} | Train Loss: {train_loss:.6f} | Val Loss: {val_loss:.6f}")

        # Overfitting warning
        if train_loss < val_loss * 0.5:
            print(f" Warning: possible overfitting")

        # Save best model
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save(model.state_dict(), SAVE_PATH)
            print(f" Saved best model (val loss: {best_val_loss:.6f})")

    print(f"\nTraining complete. Best val loss: {best_val_loss:.6f}")
    print(f"Model saved to: {SAVE_PATH}")


if __name__ == "__main__":
    main()