import os

import torch
import torch.nn as nn
from torch import distributions
import pfrl
from pfrl.nn.lmbda import Lambda
from pfrl.utils.batch_states import batch_states
from pfrl.utils.mode_of_distribution import mode_of_distribution

from agent import RSSMAgent

class PPO():
    def __init__(self, obs_size, action_size, hidden_size=256, phi=lambda x: x):
        self.phi = phi
        self.obs_normalizer = pfrl.nn.EmpiricalNormalization(
            obs_size, clip_threshold=5
        )
        self.policy = torch.nn.Sequential(
            nn.Linear(obs_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, action_size),
            pfrl.policies.GaussianHeadWithStateIndependentCovariance(
                action_size=action_size,
                var_type="diagonal",
                var_func=lambda x: torch.exp(2 * x),  # Parameterize log std
                var_param_init=0,  # log std = 0 => std = 1
            ),
        )
        self.vf = torch.nn.Sequential(
            nn.Linear(obs_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, 1),
        )
        # Combine a policy and a value function into a single model
        self.model = pfrl.nn.Branched(self.policy, self.vf)

        self.set_device(torch.device('cpu'))

    def act(self, observation):
        b_state = batch_states(observation, self.device, self.phi)
        b_state = self.obs_normalizer(b_state, update=False).type(torch.float32)
        with torch.no_grad(), pfrl.utils.evaluating(self.model):
            action_distrib, _ = self.model(b_state)
            action = mode_of_distribution(action_distrib).cpu().numpy()
        return action[0]

    def init_params(self):
        self.ortho_init(self.policy[0], gain=1)
        self.ortho_init(self.policy[2], gain=1)
        self.ortho_init(self.policy[4], gain=1e-2)
        self.ortho_init(self.vf[0], gain=1)
        self.ortho_init(self.vf[2], gain=1)
        self.ortho_init(self.vf[4], gain=1)

    def ortho_init(self, layer, gain):
        nn.init.orthogonal_(layer.weight, gain=gain)
        nn.init.zeros_(layer.bias)
    
    def load_params(self, dirname):
        self.obs_normalizer.load_state_dict(
            torch.load(os.path.join(dirname, "obs_normalizer.pt"),
            map_location=self.device)
        )
        self.model.load_state_dict(
            torch.load(os.path.join(dirname, "model.pt"),
            map_location=self.device)
        )

    def set_device(self, device):
        self.device = device
        self.obs_normalizer.to(device)
        self.model.to(device)

class SAC():
    def __init__(self, obs_size, action_size, hidden_size=256, phi=lambda x: x):
        self.obs_size = obs_size
        self.action_size = action_size
        self.hidden_size = hidden_size
        self.phi = phi
        def squashed_diagonal_gaussian_head(x):
            assert x.shape[-1] == action_size * 2
            mean, log_scale = torch.chunk(x, 2, dim=1)
            log_scale = torch.clamp(log_scale, -20.0, 2.0)
            var = torch.exp(log_scale * 2)
            base_distribution = distributions.Independent(
                distributions.Normal(loc=mean, scale=torch.sqrt(var)), 1
            )
            # cache_size=1 is required for numerical stability
            return distributions.transformed_distribution.TransformedDistribution(
                base_distribution, [distributions.transforms.TanhTransform(cache_size=1)]
            )

        self.policy = nn.Sequential(
            nn.Linear(obs_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, action_size * 2),
            Lambda(squashed_diagonal_gaussian_head),
        )

        self.q_func1 = self.make_q_func()
        self.q_func2 = self.make_q_func()

        self.set_device(torch.device('cpu'))

    def act(self, observation):
        b_state = torch.unsqueeze(batch_states(observation, self.device, self.phi), dim=0).type(torch.float32)
        with torch.no_grad(), pfrl.utils.evaluating(self.policy):
            action_distrib = self.policy(b_state)
            action = mode_of_distribution(action_distrib).cpu().numpy()
        return action[0]

    def make_q_func(self):
        q_func = nn.Sequential(
            pfrl.nn.ConcatObsAndAction(),
            nn.Linear(self.obs_size + self.action_size, self.hidden_size),
            nn.ReLU(),
            nn.Linear(self.hidden_size, self.hidden_size),
            nn.ReLU(),
            nn.Linear(self.hidden_size, 1),
        )
        return q_func
    
    def init_params(self):
        torch.nn.init.xavier_uniform_(self.policy[0].weight)
        torch.nn.init.xavier_uniform_(self.policy[2].weight)
        torch.nn.init.xavier_uniform_(self.policy[4].weight)
        torch.nn.init.xavier_uniform_(self.q_func1[1].weight)
        torch.nn.init.xavier_uniform_(self.q_func1[3].weight)
        torch.nn.init.xavier_uniform_(self.q_func1[5].weight)
        torch.nn.init.xavier_uniform_(self.q_func2[1].weight)
        torch.nn.init.xavier_uniform_(self.q_func2[3].weight)
        torch.nn.init.xavier_uniform_(self.q_func2[5].weight)

    def load_params(self, dirname):
        self.policy.load_state_dict(
            torch.load(os.path.join(dirname, "policy.pt"),
            map_location=self.device)
        )
        self.q_func1.load_state_dict(
            torch.load(os.path.join(dirname, "q_func1.pt"),
            map_location=self.device)
        )
        self.q_func2.load_state_dict(
            torch.load(os.path.join(dirname, "q_func2.pt"),
            map_location=self.device)
        )

    def set_device(self, device):
        self.device = device
        self.policy.to(device)
        self.q_func1.to(device)
        self.q_func2.to(device)

class NAVNET_SAC():
    def __init__(self, obs_size, action_size, state_size, model_dir, hidden_size=256, phi=lambda x: x):
        self.obs_size = obs_size
        self.action_size = action_size
        self.state_size = state_size
        self.hidden_size = hidden_size
        self.phi = phi

        self.navnet = RSSMAgent(model_dir)

        self.agent = SAC(state_size, action_size, hidden_size)

        self.agent.set_device(torch.device('cpu'))

    def act(self, observation):
        observation = self.state_representation(observation)
        b_state = torch.unsqueeze(batch_states(observation, self.agent.device, self.phi), dim=0).type(torch.float32)
        with torch.no_grad(), pfrl.utils.evaluating(self.agent.policy):
            action_distrib = self.agent.policy(b_state)
            action = mode_of_distribution(action_distrib).cpu().numpy()
        self.navnet.update_belief(action[0])
        return action[0]

    def state_representation(self, observation):
        lidar = observation[:self.obs_size]
        target  = observation[-3:]
        state = self.navnet(lidar)
        obs = state[0].tolist()+target
        return obs

    def load_params(self, dirname):
        self.agent.load_params(dirname)