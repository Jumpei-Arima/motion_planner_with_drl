#!/usr/bin/env python3

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

class PolNet(nn.Module):
    def __init__(self, lidar_size=36, target_size=3, action_size=2, h1=512, h2=512, h3=512):
        super(PolNet, self).__init__()
        self.fc1 = nn.Linear(lidar_size+target_size, h1)
        self.fc2 = nn.Linear(h1, h2)
        self.fc3 = nn.Linear(h2, h3)
        self.mean_layer = nn.Linear(h3, action_size)
        self.log_std_param = nn.Parameter(
            torch.randn(action_size)*1e-10 - 1)

    def forward(self, ob):
        h = F.relu(self.fc1(ob))
        h = F.relu(self.fc2(h))
        h = F.relu(self.fc3(h))
        mean = torch.tanh(self.mean_layer(h))
        log_std = self.log_std_param.expand_as(mean)
        return mean, log_std

class Policy:
    def __init__(self, model_path, min_linear_vel, max_linear_vel, max_angular_vel):
        self.net = PolNet()
        self.net.load_state_dict(torch.load(model_path, map_location="cpu"), strict=False)
        self.lb = np.array([min_linear_vel, -max_angular_vel])
        self.ub = np.array([max_linear_vel,  max_angular_vel])

    def get_action(self, state):
        state = torch.tensor(state, dtype=torch.float)
        action, _ = self.net(state).detach().cpu().numpy()
        action = self.lb + (action + 1.) * 0.5 * (self.ub - self.lb)
        action = np.clip(action, self.lb, self.ub)
        return action
