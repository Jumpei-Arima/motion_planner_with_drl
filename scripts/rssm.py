import torch
from torch import nn
from torch.nn import functional as F
from torch.distributions import Normal

class RecurrentStateSpaceModel(nn.Module):
    """
    Deterministic state model: h_t = f(h_t-1, s_t-1, a_t-1)
    Stochastic state model (prior): s_t ~ p(s_t | h_t)
    State posterior: s_t ~ q(s_t | h_t, o_t)
    """
    def __init__(self, state_size, action_size, belief_size, hidden_size, embedding_size, min_stddev=0.1):
        super().__init__()
        self.fc_state_action = nn.Linear(state_size + action_size, hidden_size)
        self.fc_belief = nn.Linear(belief_size, hidden_size)
        self.fc_state_mean_prior = nn.Linear(hidden_size, state_size)
        self.fc_state_stddev_prior = nn.Linear(hidden_size, state_size)
        self.fc_belief_embedded_obs = nn.Linear(belief_size + embedding_size, hidden_size)
        self.fc_state_mean_posterior = nn.Linear(hidden_size, state_size)
        self.fc_state_stddev_posterior = nn.Linear(hidden_size, state_size)
        self.rnn = nn.GRUCell(hidden_size, belief_size)
        self._min_stddev = min_stddev
        self.belief_size = belief_size
        self.state_size = state_size

    def forward(self, state, action, belief, embedded_next_obs):
        """
        Deterministic state model: h_t+1 = f(h_t, s_t, a_t)
        Return prior p(s_t+1 | h_t+1) and posterior p(s_t+1 | h_t+1, o_t+1)
        for model training
        """
        next_state_prior, belief = self.prior(state, action, belief)
        next_state_posterior = self.posterior(belief, embedded_next_obs)
        return next_state_prior, next_state_posterior, belief

    def prior(self, state, action, belief):
        """
        h_t+1 = f(h_t, s_t, a_t)
        Compute prior p(s_t+1 | h_t+1)
        """
        hidden = F.relu(self.fc_state_action(torch.cat([state, action], dim=1)))
        belief = self.rnn(hidden, belief)
        hidden = F.relu(self.fc_belief(belief))

        mean = self.fc_state_mean_prior(hidden)
        stddev = F.softplus(self.fc_state_stddev_prior(hidden)) + self._min_stddev
        return Normal(mean, stddev), belief

    def posterior(self, belief, embedded_obs):
        """
        Compute posterior q(s_t | h_t, o_t)
        """
        hidden = F.relu(self.fc_belief_embedded_obs(
            torch.cat([belief, embedded_obs], dim=1)))
        mean = self.fc_state_mean_posterior(hidden)
        stddev = F.softplus(self.fc_state_stddev_posterior(hidden)) + self._min_stddev
        return Normal(mean, stddev)

class Encoder(nn.Module):
    """
    Encoder to embed lidar observation to vector
    """
    def __init__(self, observation_size, embedding_size, hidden_size):
        super().__init__()
        self.fc1 = nn.Linear(observation_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, embedding_size)

    def forward(self, obs):
        hidden = F.relu(self.fc1(obs))
        hidden = F.relu(self.fc2(hidden))
        embedded_obs = F.relu(self.fc3(hidden))
        # embedded_obs = self.fc3(hidden)
        return embedded_obs

class ObservationModel(nn.Module):
    """
    p(o_t | s_t, h_t)
    Observation model to reconstruct lidar observation
    from state and rnn hidden state
    """
    def __init__(self, observation_size, state_size, belief_size, embedding_size):
        super().__init__()
        self.fc1 = nn.Linear(state_size+belief_size, embedding_size)
        self.fc2 = nn.Linear(embedding_size, embedding_size)
        self.fc3 = nn.Linear(embedding_size, observation_size)

    def forward(self, state, belief):
        hidden = F.relu(self.fc1(torch.cat([state, belief], dim=-1)))
        hidden = F.relu(self.fc2(hidden))
        observation = self.fc3(hidden)
        return observation

class LocalizationModel(nn.Module):
    """
    p(p_t | s_t, h_t)
    Localization model to estimate robot pose
    from state and rnn hidden state
    """
    def __init__(self, pose_size, state_size, belief_size, embedding_size):
        super().__init__()
        self.fc1 = nn.Linear(state_size+belief_size, embedding_size)
        self.fc2 = nn.Linear(embedding_size, embedding_size)
        self.fc3 = nn.Linear(embedding_size, pose_size)

    def forward(self, state, belief):
        hidden = F.relu(self.fc1(torch.cat([state, belief], dim=-1)))
        hidden = F.relu(self.fc2(hidden))
        pose = self.fc3(hidden)
        return pose

class RewardModel(nn.Module):
    """
    r(r_t | s_t, h_t)
    Reward model to estimate reward
    from state and rnn hidden state
    """
    def __init__(self, state_size, belief_size, goal_size, embedding_size):
        super().__init__()
        self.fc1 = nn.Linear(state_size+belief_size+goal_size, embedding_size)
        self.fc2 = nn.Linear(embedding_size, embedding_size)
        self.fc3 = nn.Linear(embedding_size, 1)

    def forward(self, state, belief, goal):
        inputs = torch.cat([state, belief], dim=-1)
        inputs = torch.cat([inputs, goal], dim=-1)
        hidden = F.relu(self.fc1(inputs))
        hidden = F.relu(self.fc2(hidden))
        reward = self.fc3(hidden)
        return reward