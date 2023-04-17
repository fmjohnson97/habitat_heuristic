import torch

from torch.utils.data import Dataset

class SubgoalFromVideoDataset(Dataset):
    def __init__(self, video_folder):
        self.video_folder = video_folder


    def __len__(self):
        pass