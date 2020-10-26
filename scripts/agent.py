import os
import json
import math

import numpy as np
import torch
from torch import nn

from rssm import Encoder, RecurrentStateSpaceModel, LocalizationModel, ObservationModel, RewardModel

class RSSMAgent(nn.Module):
    def __init__(self, model_dir, device=torch.device('cpu')):
        super(RSSMAgent, self).__init__()
        # define models
        with open(os.path.join(model_dir, 'args.txt'), 'r') as f:
            train_args = json.load(f)
        if os.path.exists(os.path.join(model_dir, 'weights', 'localization_model.pkl')):
            self.use_localization_model = True
        else:
            self.use_localization_model = False
        if os.path.exists(os.path.join(model_dir, 'weights', 'reward_model.pkl')):
            self.use_reward_model = True
        else:
            self.use_reward_model = False

        self.encoder = Encoder(
                observation_size = train_args["observation_size"],
                embedding_size = train_args["embedding_size"],
                hidden_size = train_args["encoder_hidden_size"],
                ).to(device)
        self.rssm = RecurrentStateSpaceModel(
                state_size = train_args["state_size"],
                action_size = 2,
                belief_size = train_args["belief_size"],
                hidden_size = train_args["rssm_hidden_size"],
                embedding_size = train_args["embedding_size"],
                ).to(device)
        if self.use_localization_model:
            self.localization_model = LocalizationModel(
                    pose_size = 3,
                    state_size = train_args["state_size"],
                    belief_size = train_args["belief_size"],
                    embedding_size = train_args["embedding_size"],
                    ).to(device)
        if self.use_reward_model:
            self.reward_model = RewardModel(
                    state_size = train_args["state_size"],
                    belief_size = train_args["belief_size"],
                    goal_size = 3,
                    embedding_size = train_args["embedding_size"],
                    ).to(device)
        self.obs_model = ObservationModel(
                observation_size = train_args["observation_size"],
                state_size = train_args["state_size"],
                belief_size = train_args["belief_size"],
                embedding_size = train_args["embedding_size"],
                ).to(device)
        ## load learned parameters
        self.encoder.load_state_dict(torch.load(
            os.path.join(model_dir, 'weights', 'encoder.pkl'),
            map_location=device))
        self.rssm.load_state_dict(torch.load(
            os.path.join(model_dir, 'weights', 'rssm.pkl'),
            map_location=device))
        if self.use_localization_model:
            self.localization_model.load_state_dict(torch.load(
                os.path.join(model_dir, 'weights', 'localization_model.pkl'),
                map_location=device))
            self.localization_model.eval()
        if self.use_reward_model:
            self.reward_model.load_state_dict(torch.load(
                os.path.join(model_dir, 'weights', 'reward_model.pkl'),
                map_location=device))
            self.reward_model.eval()
        self.obs_model.load_state_dict(torch.load(
            os.path.join(model_dir, 'weights', 'obs_model.pkl'),
            map_location=device))
        self.encoder.eval()
        self.rssm.eval()
        self.obs_model.eval()

        self.device = device
        self.state_size = train_args["state_size"]
        self.belief_size = train_args["belief_size"]
        self.reset()

    def reset(self):
        self.belief = torch.zeros(1, self.rssm.belief_size, device=self.device)
        self.state = torch.zeros(1, self.rssm.state_size, device=self.device)

    def forward(self, ob):
        ob = torch.unsqueeze(torch.tensor(ob, device=self.device, dtype=torch.float32), dim=0)
        with torch.no_grad():
            # encode obs
            embedded_obs = self.encoder(ob)
            # calc state posterior and sample state
            state_posterior = self.rssm.posterior(self.belief, embedded_obs)
            self.state = state_posterior.rsample()
        # output
        state_np = self.state.detach().to('cpu').numpy()
        belief_np = self.belief.detach().to('cpu').numpy()
        output = np.hstack([state_np, belief_np])
        return output

    def update_belief(self, ac):
        ac = torch.unsqueeze(torch.tensor(ac, device=self.device, dtype=torch.float32), dim=0)
        with torch.no_grad():
            # update belief
            _, self.belief = self.rssm.prior(self.state, ac, self.belief)

    def pred_pose(self, state, belief=None):
        if belief is None:
            belief = self.belief
        state = torch.unzqueeze(torch.tensor(state, device=self.device, dtype=torch.float32), dim=0)
        with torch.no_grad():
            pred_pose = self.localization_model(state, belief)
        pred_pose = pred_pose.detach().to('cpu').numpy()[0]
        return pred_pose
    
    def pred_obs(self, state, belief=None):
        if belief is None:
            belief = self.belief
        state = torch.unzqueeze(torch.tensor(state, device=self.device, dtype=torch.float32), dim=0)
        with torch.no_grad():
            pred_obs = self.obs_model(state, belief)
        pred_obs = pred_obs.detach().to('cpu').numpy()[0]
        return pred_obs