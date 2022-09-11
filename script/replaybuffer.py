"""
Last Edited by Evin Hsu 2022.07.28
-------------------------------------
Replay Buffer
"""

import random
import numpy as np

## ===========================
##  Replay Buffer
## ===========================
##  > state
##  > next state
##  > action
##  > reward
##  > done or not

# =============================================
#  Origin Replay Buffer
# =============================================
class ReplayBuffer:

    def __init__(self, s1_frame=4, s1_dim=256, a_dim=2, buffer_size=1e6, random_seed=0):
        random.seed(random_seed)        
        self.buffer_size = buffer_size 
        self.ptr = 0 
        self.count = 0 
        
        # reserve capacity 
        self.s      = np.zeros((buffer_size, s1_frame, s1_dim), dtype='float32')
        self.new_s  = np.zeros((buffer_size, s1_frame, s1_dim), dtype='float32')
        self.action = np.zeros((buffer_size, a_dim), dtype='float32')
        self.reward = np.zeros((buffer_size, 1), dtype='float32')
        self.done   = np.zeros((buffer_size, 1), dtype='int8')

    def add(self, s, a, r, new_s, done):
        
        self.s[self.ptr] = s
        self.new_s[self.ptr] = new_s
        self.action[self.ptr] = a
        self.reward[self.ptr] = r        
        self.done[self.ptr] = done
        
        self.ptr = (self.ptr + 1) % self.buffer_size
        self.count = min(self.count + 1, self.buffer_size)

    def sample_batch(self, batch_size):
        sample_size = self.count if self.count < batch_size else batch_size
        ind = np.random.choice(self.count, size=sample_size, replace=False)
        return (ind, self.s[ind], self.new_s[ind], self.action[ind], self.reward[ind], self.done[ind], None)

    def get_capacity(self):
        return self.count