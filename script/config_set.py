"""
Last Edited by Evin Hsu 2022.07.28
----------------------------------
Configuration / Parameter Setting
"""

import math

## Action Setting
action_size      = 2            ## [ v (linear) , w (angular) ]
max_action       = [1.0, 1.0]   ## [ max_v , max_w ]

## State Setting
state_class = 'lg'              ## 'g': always goal ; 'lg': serial intermidiate points & goal  ; 'glg': serial intermidiate points + goal
laser_state_size = 30           
inter_point_size = 2            ## for lg & glg
goal_size        = 2            ## for g  & glg
frame_size       = 4            ## continuous frames

## State Length
if state_class is 'g':
    state_size = laser_state_size + goal_size + action_size
elif state_class is 'lg':
    state_size = laser_state_size + inter_point_size + action_size
else:
    state_size = laser_state_size + inter_point_size + goal_size + action_size



## ================================
##  For Main
## ================================

Args = {
## Directory
    'model_path' : '/home/user/rrt_rl_review_ws/src/rrt_rl_nav/data/model',         ## Model save path
    'train_file' : 'SAC_model_20227811550/SAC_model_20227811550',                   ## Load model for training (directory/file name)
    'eval_file'  : 'SAC_model_2022616171231/SAC_model_2022616171231',               ## Load model for evolution (directory/file name)
    'real_file'  : 'SAC_model_20227444530/SAC_model_20227444530',                   ## Load model for real world (directory/file name)
    'loss_path'  : '/home/user/rrt_rl_review_ws/src/rrt_rl_nav/data/loss',          ## Loss record save path
    'perf_path'  : '/home/user/rrt_rl_review_ws/src/rrt_rl_nav/data/train',         ## Performance of training record save path
    'eval_path'  : '/home/user/rrt_rl_review_ws/src/rrt_rl_nav/data/eval',          ## Performance of evolution record save path
## Function Switch
    'real'  : False,                            ## real evo. mode (T)
    'train' : True,                             ## sim. train mode (T) / sim evo. mode (F)
    'debug' : False,
    'back_enable' : True,
    'model_load' : False,
    'scene' : 'dynamic_7_4_2',                  ## used scene   (in scene_set.py)
    'robot' : 'ttb',                            ## robot setting mode (in scene_set.py) [ ttb, youbot, ttb_like_youbot ]
## record saving
    'model_save' : True, 
    'check_save' : True,                        ## checkpoint
    'loss_save'  : True,
    'perf_save'  : True,
    'check_step' : 100,                         ## checkpoint saving interval (episode)
## for simulation training
    'use_network' : 'SAC',
    'a_lr' : 5e-4,  # 1e-5                      ## Actor Learning Rate
    'c_lr' : 5e-4,  # 5e-6                      ## Critic Learning Rate
    'max_action' : max_action,                  ## [ max_v (linear), max_w (angular) ] ; min_v = -max_v if back_enable else 0 ; min_w = -max_w
    'state_size' : state_size,
    'frame_size' : frame_size,
    'action_size': action_size, 
    'batch_size' : int(32),
    'max_all_step' : 2e5,
    'warm_up' : 250,                            ## start training
## for simulation evolution
    'eval_save' : True,                         ## result saving
    'eval_episode' :50, 
    'eval_step' : 300,
## for real evolution
    'real_step' : 400,
## Replay Buffer 
    'use_replay'  : 'origin', 
    'buffer_seed' : int(0),                     ## random seed
    'buffer_size' : int(5e5), 
## SAC Hyperparameter / Paremeter
    'sac_gamma' : float(0.9), 
    'sac_tau' : float(0.005),  
    'sac_alpha' : float(0.2),
    'sac_policy' : 'Gaussian', 
    'sac_policy_freq' : int(3), 
    'sac_auto_entropy_tuning' : False, 
    'sac_epsilon' : int(1e-6),
    'sac_policy_noise' : int(1),     
}

EnvArgs = {
    'debug' : False,
    'checker' : True,                           ## Check Image Publish (Laser Scan & Map) (ROS)
    'use_global' : True,
    'set_init_yaw' : True,
    'state_class' : state_class,
## about laser state
    'scan_topic' : "/scan",                     ## (ROS)
    'odom_topic' : "/odom",                     ## (ROS)
    'scan_angle' : 240,                         ## (-scan_angle/2 ← 0 → scan_angle/2)
    'laser_data_size' : laser_state_size, 
## about action
    'state_freq' : -1,                          ## Action execute delay (s), unused if less than < 0
    'robot_wrong' : 0.25,                       ## restart of simulation (roll)
## distance threshold for training
    'goal_dist_tol' : 0.3,                      ## goal position arriving
    'goal_angle_tol' : 15 * math.pi / 180,      ## goal angle arriving
    'inter_point_tol' : 0.5,                    ## inter points arriving (optional)
    'collision_dist' : 0.15,                    ## collision 
## distance threshold for evolution
    'eval_goal_tol' : 0.1,
    'eval_angle_tol' : 15 * math.pi / 180,
    'eval_inter_tol' : 0.5,
    'eval_coll_dist' : 0.1, 
## reward
    'arrived_R'   : 60,
    'collision_R' : -80,
    'inter_R'    : 10.,    
}

## =====================================
##  For Network Model
## =====================================

lstm_out = 128
ModelArgs = {
## Layer Dimension
    'action_ch' : action_size,
    'lstm_ch' : [ state_size, lstm_out, 1],
    'line_ch' : [   lstm_out, \
                    256, \
                    512, \
                    512, \
                    256, \
                    64,  \
                    action_size],
## Dropout
    'a_dp' : 0.2,
    'c_dp' : 0.2,
    'lstm_dp' : 0.,
# Initial Weight
    'init_w' : 3e-3,
## SAC Setting
    'LOG_SIG_MIN' : -20,
    'LOG_SIG_MAX' : 2,
}
