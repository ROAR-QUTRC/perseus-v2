import torch


def main():
    # Set num of threads
    torch.set_num_threads(7)

    # Device setup
    device = torch.device()
