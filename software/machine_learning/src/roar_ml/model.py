import torch.nn as nn


class IlmeniteModel(nn.Module):
    def __init__(self):
        super().__init__()

        layers = []

        # First Layer
        layers.append(nn.Linear(18, 64))
        layers.append(nn.ReLU())
        layers.append(nn.Dropout(0.2))

        # Second Layer
        layers.append(nn.Linear(64, 32))
        layers.append(nn.ReLU())
        layers.append(nn.Dropout(0.2))

        # Last Layer
        layers.append(nn.Linear(32, 1))
        layers.append(nn.Sigmoid())

        # Apply our layers
        self.network = nn.Sequential(*layers)

    def forward(self, x):
        return self.network(x)
