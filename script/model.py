"""
Last Edited by Evin Hsu 2022.07.28
----------------------------------------
RL Neural Network Model Structure
"""

import math
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal
from config_set import ModelArgs

torch.manual_seed(0)
torch.cuda.manual_seed(0)
torch.cuda.manual_seed_all(0)

def weights_init_(m):
    if isinstance(m, nn.Linear):
        stdv = 1. / math.sqrt(m.weight.size(1))
        m.weight.data.uniform_(-stdv, stdv)
        if m.bias is not None:
            torch.nn.init.constant_(m.bias, 0)


## =======================================
##  Critic Network
## =======================================

class Critic(nn.Module):
    """
    Input:  State Tensor & Action Tensor
    Output: Q value
    """
    def __init__(self, device):
        super(Critic, self).__init__()
        
        # s ------------------------
        self.lstm  = nn.LSTM(ModelArgs['lstm_ch'][0], ModelArgs['lstm_ch'][1], ModelArgs['lstm_ch'][2], batch_first=True, dropout=0.)
        self.s_linear1 = nn.Linear(ModelArgs['lstm_ch'][1] , ModelArgs['line_ch'][2])
        self.s_linear2 = nn.Linear(ModelArgs['line_ch'][2] , ModelArgs['line_ch'][4])
        # a -----------------------
        self.a_linear1 = nn.Linear(ModelArgs['action_ch'], ModelArgs['line_ch'][2])
        self.a_linear2 = nn.Linear(ModelArgs['line_ch'][2] , ModelArgs['line_ch'][4])
        # concatenate -------------
        self.concat = nn.Linear(ModelArgs['line_ch'][4], 1)
        
        self.to(device)
        self.apply(weights_init_)
        self.concat.bias.data.uniform_(-ModelArgs['init_w'], ModelArgs['init_w'])
    
    
    def forward(self, s, a):
        _, (s, _) = self.lstm(s)
        s = s[-1]
        s1 = self.s_linear1(s)
        s1 = self.s_linear2(s1)
        a1 = self.a_linear1(a)
        a1 = self.a_linear2(a1)
        
        s11 = torch.mm(s, self.s_linear1.weight.data.t())
        s11 = torch.mm(s11, self.s_linear2.weight.data.t())
        s12 = torch.mm(a, self.a_linear1.weight.data.t())
        s12 = torch.mm(s12, self.a_linear2.weight.data.t())
        s = F.relu(s11 + s12 + self.a_linear2.bias.data)    
        
        q1 = self.concat(s)
        return q1


## =======================================
##  Policy Network
## =======================================

## Actor Network
class GaussianPolicy(nn.Module):
    def __init__(self, device, max_action = None):
        super(GaussianPolicy, self).__init__()
        
        self.lstm  = nn.LSTM(ModelArgs['lstm_ch'][0], ModelArgs['lstm_ch'][1], ModelArgs['lstm_ch'][2], batch_first=True, dropout=ModelArgs['a_dp'])
        self.linear1 = nn.Linear(ModelArgs['line_ch'][0], ModelArgs['line_ch'][1])
        self.linear2 = nn.Linear(ModelArgs['line_ch'][1], ModelArgs['line_ch'][2])
        self.linear3 = nn.Linear(ModelArgs['line_ch'][2], ModelArgs['line_ch'][3])
        self.linear4 = nn.Linear(ModelArgs['line_ch'][3], ModelArgs['line_ch'][4])
        self.mean    = nn.Linear(ModelArgs['line_ch'][4], ModelArgs['action_ch'])
        self.log_std = nn.Linear(ModelArgs['line_ch'][4], ModelArgs['action_ch'])
        
        self.dropout = nn.Dropout(p=ModelArgs['a_dp'])
        self.to(device)
        self.apply(weights_init_)
        self.mean.bias.data.uniform_(-ModelArgs['init_w'], ModelArgs['init_w'])
        self.log_std.bias.data.uniform_(-ModelArgs['init_w'], ModelArgs['init_w'])
        
        if max_action is None:
            self.action_scale = torch.tensor(1.).to(device)
            self.action_bias = torch.tensor(0.).to(device)
        else:
            min_action = -max_action
            self.action_scale = torch.tensor((max_action - min_action) / 2.).to(device)
            self.action_bias = torch.tensor((max_action + min_action) / 2.).to(device)
    
    def forward(self, s):
        self.lstm.flatten_parameters()
        # LSTM
        _, (s, _) = self.lstm(s)
        s = s[-1]
        # Linear
        out = F.relu(self.linear1(s))
        out = F.relu(self.linear2(out))
        out = F.relu(self.linear3(out))
        out = torch.tanh(self.linear4(out)) 
        mean = self.mean(out)
        log_std = self.log_std(out)
        log_std = torch.clamp(log_std, min=ModelArgs['LOG_SIG_MIN'], max=ModelArgs['LOG_SIG_MAX'])
        return mean.to(torch.float32), log_std.to(torch.float32)
    
    
    def sample(self, s):
        mean, log_std = self.forward(s)
        std = torch.exp(log_std)
        normal = Normal(mean, std)
        mean = torch.tanh(mean) * self.action_scale + self.action_bias
        x_t = normal.rsample()
        log_probs = normal.log_prob(x_t).sum(dim=-1, keepdim=True)
        log_probs -= (2 * (np.log(2) - x_t - F.softplus(-2 * x_t))).sum(dim=-1, keepdim=True)
        x_t = torch.tanh(x_t)
        action = x_t * self.action_scale + self.action_bias
        return action.to(torch.float32), log_probs.to(torch.float32), mean.to(torch.float32)