import torch.nn as nn


class IlmeniteModel(nn.Module):
    def __init__(self):
        super().__init__()

        layers = []

        # First Layer
        #reduced the first layer to 32 because there's not much testing data
        layers.append(nn.Linear(7, 32)) #32 was originally 64
        layers.append(nn.ReLU())
        layers.append(nn.Dropout(0)) #changed dropout to be 0.1 instead of 0.2
        #dropout prevents overfitting 

        # Second Layer
        layers.append(nn.Linear(32, 16))
        layers.append(nn.ReLU())
        layers.append(nn.Dropout(0))

        # Last Layer
        layers.append(nn.Linear(16, 1))
        layers.append(nn.Sigmoid())

        # Apply our layers
        self.network = nn.Sequential(*layers)

    def forward(self, x):
        return self.network(x)
