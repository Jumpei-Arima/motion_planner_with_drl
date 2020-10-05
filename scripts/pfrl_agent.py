import torch
import torch.nn as nn
import pfrl
from pfrl.agents import PPO

class pfrl_ppo_agent:
    def __init__(self, model_path, obs_size=39, action_size=2):
        obs_size = obs_size
        action_size = action_size

        obs_normalizer = pfrl.nn.EmpiricalNormalization(
            obs_size, clip_threshold=5
        )
        policy = torch.nn.Sequential(
            nn.Linear(obs_size, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, action_size),
            pfrl.policies.GaussianHeadWithStateIndependentCovariance(
                action_size=action_size,
                var_type="diagonal",
                var_func=lambda x: torch.exp(2 * x),  # Parameterize log std
                var_param_init=0,  # log std = 0 => std = 1
            ),
        )
        vf = torch.nn.Sequential(
            nn.Linear(obs_size, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 1),
        )
        model = pfrl.nn.Branched(policy, vf)
        opt = torch.optim.Adam(model.parameters(), lr=3e-4, eps=1e-5)
        self.agent = PPO(model, opt, obs_normalizer=obs_normalizer)
        self.agent.load(model_path)

    def get_action(self, state):
        state = torch.tensor(state, dtype=torch.float)
        action = self.agent.act(state)
        return action

