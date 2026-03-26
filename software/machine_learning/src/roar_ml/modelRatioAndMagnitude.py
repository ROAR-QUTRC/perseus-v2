import torch.nn as nn


class IlmeniteModel(nn.Module):
    def __init__(self):
        super().__init__()
        layers = []
        layers.append(nn.Linear(60, 64))
        layers.append(nn.ReLU())
        layers.append(nn.Dropout(0.05))
        #0.05 is pretty good
        #tried 0.1, 0.04, 0.03

        layers.append(nn.Linear(64, 32))
        layers.append(nn.ReLU())
        layers.append(nn.Dropout(0.05))

        layers.append(nn.Linear(32, 16))
        layers.append(nn.ReLU())
        layers.append(nn.Dropout(0.05))


        layers.append(nn.Linear(16, 1))
        layers.append(nn.Sigmoid())

        self.network = nn.Sequential(*layers)

    def forward(self, x):
        return self.network(x)