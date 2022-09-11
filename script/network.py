"""
Last Edited by Evin Hsu 2022.07.28
--------------------------------------
RL Neural Network Functions
"""

import numpy as np
import torch
import torch.nn.functional as F
import torch_optimizer as optim
import copy
from model import Critic, GaussianPolicy

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
torch.backends.cudnn.enabled=False

np.random.seed(0)
torch.manual_seed(0)
torch.cuda.manual_seed(0)
torch.cuda.manual_seed_all(0)

class SAC:
    
    def __init__(self,  max_action, 
                        actor_lr=1e-3, 
                        critic_lr=1e-3, 
                        gamma=0.99, 
                        tau=0.005, 
                        alpha=0.2, 
                        policy='Gaussian', 
                        policy_freq=2, 
                        auto_tuning=False, 
                        debug=False, 
                        enable_backward=False):
        
        print(f'\nSAC model.')
        print(f'Use device : {device}')

        self.debug = debug
        self.enable_backward = enable_backward
        self.gamma = gamma
        self.tau = tau
        self.alpha = alpha
        self.target_update_interval = policy_freq
        self.auto_entropy_tuning = auto_tuning
        self.max_action = torch.from_numpy(max_action).to(device)
        self.iter = 0
        
        
        ## Critic Network
        self.critic1 = Critic(device)
        self.critic1_target = Critic(device)
        self.critic1_target.load_state_dict(self.critic1.state_dict())
        self.critic1_optimizer = torch.optim.Adam(self.critic1.parameters(), lr = critic_lr)
        self.critic2 = Critic(device)
        self.critic2_target = Critic(device)
        self.critic2_target.load_state_dict(self.critic2.state_dict())
        self.critic2_optimizer = torch.optim.Adam(self.critic2.parameters(), lr = critic_lr)

        ## Policy Network
        if self.auto_entropy_tuning:
            self.target_entropy = -torch.prod(torch.tensor(max_action).to(device)).item()
            self.log_alpha = torch.zeros(1, requires_grad=True, device=device)
            self.alpha_optim = torch.optim.Adam([self.log_alpha], lr = actor_lr)
        self.policy = GaussianPolicy(device, max_action).to(device)
        self.policy_optimizer = torch.optim.Adam(self.policy.parameters(), lr = actor_lr)

    def get_action(self, state, is_train):
        state= torch.tensor(state).to(device).to(torch.float32)
        if is_train:
            action, _, _ = self.policy.sample(state)
        else:
            _, _, action = self.policy.sample(state)
        action = np.squeeze(action.cpu().detach().cpu().numpy())
        return action

    def train(self, replay_buffer, iteration=1, batch_size=64, use_replay="origin"):

        self.iter += 1
        loss_set = []
        for _ in range(iteration):
            
            loss = {
                'critic1_loss' : 0.,
                'critic2_loss' : 0.,
                'policy_loss' : 0.,
            }

            ind, state, next_state, b_action, reward, done, weight = replay_buffer.sample_batch(batch_size)

            state      = torch.tensor(state).to(device).to(torch.float32)
            next_state = torch.tensor(next_state).to(device).to(torch.float32)
            action     = torch.tensor(b_action).to(device).to(torch.float32)
            reward     = torch.tensor(reward).to(device).to(torch.float32).reshape(-1, 1)
            done       = torch.tensor(done).to(device).reshape(-1, 1)

            reward = 25*(reward - reward.mean(dim=0)) /reward.std(dim=0)

            with torch.no_grad():
                next_action, next_log_pi, _  = self.policy.sample(next_state)
                target_Q1 = self.critic1_target(next_state, next_action)
                target_Q2 = self.critic2_target(next_state, next_action)
                target_Q = torch.min(target_Q1, target_Q2) - self.alpha * next_log_pi
                target_Q = reward + (1 - done) * self.gamma * target_Q

            current_Q1 = self.critic1(state, action)
            current_Q2 = self.critic2(state, action)

            loss1 = F.mse_loss(current_Q1, target_Q) 
            loss2 = F.mse_loss(current_Q2, target_Q)

            self.critic1_optimizer.zero_grad()
            loss1.backward()
            self.critic1_optimizer.step()
            
            self.critic2_optimizer.zero_grad()
            loss2.backward()
            self.critic2_optimizer.step()
  
            pi, log_pi, _ = self.policy.sample(state)
            q1_pi = self.critic1(state, pi)
            q2_pi = self.critic2(state, pi)
            min_q_pi = torch.min(q1_pi, q2_pi)
            policy_loss = ((self.alpha * log_pi) - min_q_pi).mean()

            self.policy_optimizer.zero_grad()
            policy_loss.backward()
            self.policy_optimizer.step() 
            
            if self.debug:
                print('\n, loss = %.4f, %.4f, %.4f' % (loss1, loss2, policy_loss), end=" ")
            
            loss['critic1_loss'] = loss1.item()
            loss['critic2_loss'] = loss2.item()
            loss['policy_loss'] = policy_loss.item()
            if not all(x==0 for x in loss.values()):
                loss_set.append(loss)

            if self.auto_entropy_tuning:
                alpha_loss = -(self.log_alpha * (log_pi + self.target_entropy).detach()).mean()
                
                self.alpha_optim.zero_grad()
                alpha_loss.backward()
                self.alpha_optim.step()
                
                self.alpha = self.log_alpha.exp()
                alpha_tlogs = self.alpha.clone()
            else:
                alpha_loss = torch.tensor(0.).to(device)
                alpha_tlogs = torch.tensor(self.alpha)

            if self.iter % self.target_update_interval == 0:
                for param, target_param in zip(self.critic1.parameters(), self.critic1_target.parameters()):
                    target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)
                for param, target_param in zip(self.critic2.parameters(), self.critic2_target.parameters()):
                    target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)
        
        return loss_set

    def save(self, filename, directory):
        torch.save(self.policy.state_dict(), '%s/%s_actor.pth' % (directory, filename))
        torch.save(self.critic1.state_dict(), '%s/%s_critic1.pth' % (directory, filename))
        torch.save(self.critic2.state_dict(), '%s/%s_critic2.pth' % (directory, filename))

    def load(self, filename, directory):
        self.policy.load_state_dict(torch.load('%s/%s_actor.pth' % (directory, filename)))
        self.actor_target = copy.deepcopy(self.policy)
        self.critic1.load_state_dict(torch.load('%s/%s_critic1.pth' % (directory, filename)))
        self.critic1_target = copy.deepcopy(self.critic1)
        self.critic2.load_state_dict(torch.load('%s/%s_critic2.pth' % (directory, filename)))
        self.critic2_target = copy.deepcopy(self.critic2)

    def checkpoint_save(self, epoch, filename, directory):
        torch.save(self.policy.state_dict(), '%s/%s_ckpt_%s_actor.pth' % (directory, filename, str(epoch)))
        torch.save(self.critic1.state_dict(), '%s/%s_ckpt_%s_critic1.pth' % (directory, filename, str(epoch)))
        torch.save(self.critic2.state_dict(), '%s/%s_ckpt_%s_critic2.pth' % (directory, filename, str(epoch)))