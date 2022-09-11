
"""
Last Edited by Evin Hsu 2022.07.28
------------------------------------
Main
"""
 

#! /usr/bin/env python3
import os
import time
import argparse
import numpy as np
import rospy

from config_set import Args
from scene_set import SceneDict, StepDict
from dynamic_env import DynamicEnv
from real_env import RealEnv
from network import SAC
from replaybuffer import ReplayBuffer, PER
from env_utils import save_dict_to_csv

## select mode by command line : train (simulation), evolution (simulation), evolution (real)
parser = argparse.ArgumentParser()
parser.add_argument("--train", action="store_true")
parser.add_argument("--eval", action="store_true")
parser.add_argument("--real", action="store_true")
a = parser.parse_args()
if a.real:
    Args['real'] = True
    Args['train'] = False
elif a.eval:
    Args['real'] = False
    Args['train'] = False 
else:
    Args['real'] = False
    Args['train'] = True

"""Evolution (real)
"""
def real():

    max_action = np.array(Args['max_action'])
    max_step = Args['real_step']
    agent = SAC(max_action, 
                Args['a_lr'], 
                Args['c_lr'], 
                Args['sac_gamma'], 
                Args['sac_tau'], 
                Args['sac_alpha'], 
                Args['sac_policy'], 
                Args['sac_policy_freq'],
                Args['sac_auto_entropy_tuning'],
                Args['debug'], 
                Args['back_enable'])
    agent.load(Args['real_file'], Args['model_path'])
    ## Environment Initialization
    envs = RealEnv(Args['robot'], Args['back_enable'])
    time.sleep(2.0)

## ===================================
##              Episodes 
## ===================================

    print(f'\n====================================\nStart\n====================================')
    
    while not rospy.is_shutdown():
        
        ## Step 1. Reset and Waiting for the specified goal from RViz 2D Nav Goal
        
        if envs.done:
        
            envs.robot_cmd(0., 0.)
            envs.global_plan = False
            envs.local_plan = False
            
            global_time = 0
            local_time = 0
            reward = 0
            step = 0
            col, done, arrive_pos, arrive_ang, restart = False, False, False, False, False      ## 碰撞、任務完成、抵達終點、抵達目標角度、重來 ( e.g. 光達接收資訊出錯 )
            action = np.array([0., 0.])
            
            envs.done = False
        
        ## Step 2. Global Planning
        
        elif envs.global_plan:
        
            print('\n\nGlobal Planning')
            t_start = time.time()
            using_time = time.time()
            
            ret = envs.get_global_path()
            envs.global_plan = False
            if ret:     ## success
                global_time = time.time() - using_time
                print('>> Success.\n')
                
                ## Local Planning Prepare
                new_state, _, _, _, restart = envs.step(action, True)   
                while not rospy.is_shutdown() and restart:
                    print("Initial State Failed. Retry.")
                    new_state, _, _, _, restart = envs.step(action, True)
                ss_seq = [new_state] * Args['frame_size']
                ss_stack = np.array(ss_seq)
                
                print('\n > Local Planning')
                envs.local_plan = True
                using_time = time.time()
                rospy.sleep(5)
            else:
                print('>> Failed.\n')
        
        ## Step 3. Local Planning
        
        elif envs.local_plan:
            if step > max_step:     ## Terminate
                envs.done = True
                continue

            new_action = agent.get_action(ss_stack[np.newaxis], False)
            new_action[0] = np.clip(new_action[0], -max_action[0], max_action[0])
            new_action[1] = np.clip(new_action[1], -max_action[1], max_action[1])

            new_state, col, arrive_pos, rr, restart = envs.step(new_action, False)
            
            while not rospy.is_shutdown() and restart:
                print("Update State Failed. Retry.")
                new_state, col, arrive_pos, rr, restart = envs.step(new_action, False)
                
            del ss_seq[-1]
            ss_seq.insert(0, new_state)
            new_ss_stack = np.stack(ss_seq, axis = 0)
            reward += rr
            
            done = arrive_pos or col
            done = True if step + 1 > max_step else done
            
            print('Step %2d, action = (%.2f, %.2f), reward = (%.2f). ' \
                    % (step+1, new_action[0], new_action[1], rr))
            
            if done:
                envs.robot_cmd(0, 0)
                if col:          print(' >> Collision Occured.')
                elif arrive_pos: print(' >> Arrived Goal.')
                else:            print(' >> Over step.')

                ##  Conclusion
                local_time = time.time() - t_start
                using_time = time.time() - using_time
                print("\n-----------------------------------------\n  Conclusion\n-----------------------------------------")
                print(Args['real_file'])
                print('Max action = (%.2f, %.2f). %s Backward.' % (max_action[0], max_action[1], ('Enable' if Args['back_enable'] else 'Disable')))
                print(f'Total execution time = {using_time}.')
                print(f'Total global planning time = {global_time}.')
                print(f'Total local planning time = {local_time}.')
                print(f'Total step = {step}.')
                print(f'Total reward = {reward}.\n')
                
                envs.local_plan = False
                envs.done = True
                continue
            
            action = new_action
            ss_stack = new_ss_stack
            step += 1
        
        else:
            print("\r>> Select Goal by RViz \'2D Nav Goal\'  ", end='')

"""Train (simulation)
"""
def train():

    max_action = np.array(Args['max_action'])
    max_all_step = Args['max_all_step']
    agent = SAC(max_action, 
                Args['a_lr'], 
                Args['c_lr'], 
                Args['sac_gamma'], 
                Args['sac_tau'], 
                Args['sac_alpha'], 
                Args['sac_policy'], 
                Args['sac_policy_freq'],
                Args['sac_auto_entropy_tuning'],
                Args['debug'], 
                Args['back_enable'])
    if Args['model_load']:
        agent.load(Args['train_file'], Args['model_path'])
        
    ## Create Directory
    if Args['model_save'] or Args['check_save']:
        now = time.localtime()
        save_file = ''.join(('SAC_model_' ,str(now.tm_year), str(now.tm_mon),str(now.tm_mday),str(now.tm_hour), str(now.tm_min), str(now.tm_sec)))
        save_path = os.path.join(Args['model_path'], save_file)
        try:
            os.mkdir(save_path)
        except:
            print('Failed to create model folder.')

    if Args['use_replay'] == 'per':
        replay_buffer = PER(Args['frame_size'],
                            Args['state_size'],
                            Args['action_size'],
                            Args['per_alpha'],
                            Args['per_beta'],
                            Args['per_beta_sch'],
                            Args['buffer_size'], 
                            Args['buffer_seed'])
    else:
        replay_buffer = ReplayBuffer(Args['frame_size'],
                                    Args['state_size'],
                                    Args['action_size'], 
                                    Args['buffer_size'], 
                                    Args['buffer_seed'])
                                    
    ## Environment Initialization
    if Args['scene'] in SceneDict:
        envs = DynamicEnv(Args['scene'], Args['robot'], Args['back_enable'], True)
    else:
        print('\nScene non-exist!!! Terminate!!!')
        return -1
    _ = envs.scene_initial()
    envs.robot_cmd(0, 0)    # stop robot
    time.sleep(2.0)
    
    if Args['scene'] in StepDict:
        step_decay = StepDict[Args['scene']]
    else:
        step_decay = StepDict['default']
    step_decay.append([ Args['max_all_step']+10, 0 ])
    StepDict.clear()
    max_step = step_decay[0][1]
    del step_decay[0]
    
## ===================================
##              Episodes 
## ===================================
    
    done = True         ## episode done 
    restart = False     ## episode discard & restart
    episode = -1        ## current episode
    total_step = 0      ## total step
    done_count = 0      ## count of success episode
    loss_set = []       ## loss record
    episode_set = []    ## performance record
    train_time = time.time()    ## total used time
    
    while not rospy.is_shutdown() and total_step < max_all_step:
        
        if done:    ## conclusion & restart
            
            episode += 1
            envs.robot_cmd(0, 0)    # stop robot
            
            ## conclusion
            if episode > 0:
                
                total_step += step
                done_count = (done_count + 1) if arrive_pos else done_count
                
                print('Episode %3d (%d/%d) : steps = %3d, reward = %.2f  ' % (episode, total_step, max_all_step, step + 1, reward), end='')
                if arrive_pos: print(' >> Arrived Goal.')
                elif col:      print(' >> Collision Occured.')
                else:          print(' >> Over step.')
                
                for exp in experience:
                    replay_buffer.add(exp[0], exp[1], exp[2], exp[3], exp[4])
                
                if replay_buffer.get_capacity() >= Args['warm_up']:
                    ## Training
                    loss = agent.train(replay_buffer, step+1, Args['batch_size'], Args['use_replay']) 
                    ## recoed
                    loss_set = loss_set + loss
                    if Args['check_save'] and (episode) % Args['check_step'] == 0:
                        try:
                            agent.checkpoint_save(episode, save_file, save_path)
                        except:
                            print("Fail to save checkpoint.")
                
                ## record
                performance['step'] = step+1
                performance['local_time'] += time.time() - t_start
                performance['reward'] = reward
                if arrive_pos:
                    performance['done'] = 1
                elif col:
                    performance['done'] = -1
                else:   # over step
                    performance['done'] = 0
                episode_set.append(performance)
        
        ## restart
        if done or restart: 
            
            envs.robot_cmd(0, 0)    # stop robot
            
            col, done, arrive_pos, arrive_ang, restart = False, False, False, False, False 
            experience = []     ## replay buffer experience
            performance = {
                'episode': episode,
                'step': 0.,
                'reward': 0.,
                'global_time': 0.,
                'local_time': 0.,  
                'done': 0,         
            }
            ## update step limitation (in scene.py)
            if total_step > step_decay[0][0]:
                max_step = step_decay[0][1]
                if len(step_decay) > 0:
                    del step_decay[0]
                envs.set_init_raw = False
            
            ## Global planning (random start & goal)
            t_start = time.time()
            if not envs.get_global_path():
                restart = True
                continue
            performance['global_time'] = time.time() - t_start
            
            ## Local Planning (navigation)
            reward = 0
            step = 0
            action = np.array([0., 0.])
                      
            ## State Initialization
            new_state, _, _, _, restart = envs.step(action, True)
            if restart:
                continue
            ss_seq = [new_state] * Args['frame_size']
            ss_stack = np.array(ss_seq)
            
            t_start = time.time()
            
        ## ===================================
        ##               Steps
        ## ===================================
        
        new_action = agent.get_action(ss_stack[np.newaxis], True)
        new_action[0] = np.clip(new_action[0], -max_action[0], max_action[0])
        new_action[1] = np.clip(new_action[1], -max_action[1], max_action[1])
        
        new_state, col, arrive_pos, rr, restart = envs.step(new_action, False)
        if restart:  
            continue
        del ss_seq[-1]
        ss_seq.insert(0, new_state)
        new_ss_stack = np.stack(ss_seq, axis = 0)
        reward += rr
        
        done = arrive_pos or col
        done = True if step + 1 > max_step else done
        
        experience.append([ss_stack, new_action, rr, new_ss_stack, int(done)])
        
        action = new_action
        ss_stack = new_ss_stack
        
        step += 1
        
    ## ===================================
    ##             Conclusion
    ## ===================================
    
    envs.del_all()
    envs.sim_pause()
    
    train_time = time.time() - train_time
    reward_avg = round(sum([per['reward'] for per in episode_set]) / len(episode_set) , 2)
    global_time_avg = round(sum([per['global_time'] for per in episode_set]) / len(episode_set) , 2)
    local_time_avg = round(sum([per['local_time'] for per in episode_set]) / len(episode_set) , 2)
    
    print('\n')    
    if Args['model_save']:
        agent.save(save_file, save_path)
        print(f'Save Model at {save_path}/{save_file}.')
    if Args['loss_save']:
        save_dict_to_csv(Args['loss_path'], save_file, loss_set)
    if Args['perf_save']:
        save_dict_to_csv(Args['perf_path'], save_file, episode_set)
    
    print('\n------------------------------------------------------------')
    print(save_file)
    print('Scene : ', Args['scene'])
    print('Max action = (%.2f, %.2f). %s Backward.' % (max_action[0], max_action[1], ('Enable' if Args['back_enable'] else 'Disable')))
    print(f'Total execution time = {train_time}.')
    print(f'Average global planning time = {global_time_avg}.')
    print(f'Average local planning time = {local_time_avg}.')
    print(f'Average reward = {reward_avg}.')
    print(f'count of arrive goal = {done_count}/{episode}.')


""" Evolution (simulation)
"""
def eval():

## ===================================
##          Initialization
## ===================================
    max_action = np.array(Args['max_action'])    
    agent = SAC(max_action, 
                Args['a_lr'], 
                Args['c_lr'], 
                Args['sac_gamma'], 
                Args['sac_tau'], 
                Args['sac_alpha'], 
                Args['sac_policy'], 
                Args['sac_policy_freq'],
                Args['sac_auto_entropy_tuning'],
                Args['debug'], 
                Args['back_enable'])
    agent.load(Args['eval_file'], Args['model_path'])
    if Args['scene'] in SceneDict:
        envs = DynamicEnv(Args['scene'], Args['robot'], Args['back_enable'], False)
    else:
        print('\nScene non-exist!!! Terminate!!!')
        return -1
    _ = envs.scene_initial()
    envs.robot_cmd(0, 0)    # stop robot
    time.sleep(2.0)
    
## ===================================
##              Episodes 
## ===================================
    max_episode = Args['eval_episode']
    max_step = Args['eval_step']
    episode = 0
    
    global_time = 0
    local_time = 0
    local_step = 0
    done_count = 0
    reward_count = 0
    
    episode_set = []
    
    using_time = time.time()
    
    
    while not rospy.is_shutdown() and episode < max_episode:
        
        print(f'\n====================================\nEpisode {episode+1}\n====================================')
        
        envs.robot_cmd(0, 0)    # stop robot
        col, done, arrive_pos, arrive_ang, restart = False, False, False, False, False
        envs.set_init_raw = False
        
        ## Global Planning
        print('\n > Global Planning', end='')
        t_start = time.time()
        if not envs.get_global_path():
            restart = True
            print("\n >> Failed.")
            continue
        global_time += time.time() - t_start
        print("\n >> Done.")
        
        ## Local Planner
        print('\n > Local Planning')
        reward = 0
        step = 0
        action = np.array([0., 0.])
        
        new_state, _, _, _, restart = envs.step(action, True)   # Initial Condition
        if restart:
            print("Robot has trouble. Restart.")
            continue
        ss_seq = [new_state] * Args['frame_size']
        ss_stack = np.array(ss_seq)
        
        print()
        
        if Args['eval_save']:
            performance = {
                'episode': episode+1,
                'step': 0,
                'reward': 0,
                'time': 0,
                'arrived': 0,
                'collision': 0,
            }
        
        t_start = time.time()
        
        ## ===================================
        ##               Steps 
        ## ===================================
        
        while not rospy.is_shutdown() and step < max_step:
            new_action = agent.get_action(ss_stack[np.newaxis], False)
            
            new_action[0] = np.clip(new_action[0], -max_action[0], max_action[0])
            new_action[1] = np.clip(new_action[1], -max_action[1], max_action[1])
            
            if not Args['back_enable']:
                new_action[0] = abs(new_action[0])
            
            ## Update environment
            new_state, col, arrive_pos, rr, restart = envs.step(new_action, False)
            if restart:
                print("Robot has trouble. Restart.")
                break
            del ss_seq[-1]
            ss_seq.insert(0, new_state)
            new_ss_stack = np.stack(ss_seq, axis = 0)
            reward += rr
            
            ## Done an episode
            done = arrive_pos or col
            done = True if step + 1 > max_step else done
            
            print('Step %2d, action = (%.2f, %.2f), reward = (%.2f). ' \
                    % (step+1, new_action[0], new_action[1], rr))
            
            if done:
                break
            
            action = new_action
            ss_stack = new_ss_stack
            step += 1
        
        ## ===================================
        ##           End of Episode
        ## ===================================
        envs.robot_cmd(0, 0)    ## stop robot
        if not restart:         ## Episode Conclusion 
            local_time += time.time() - t_start
            done_count = (done_count + 1) if arrive_pos else done_count
            reward_count += reward
            local_step += step
            episode += 1
            
            performance['step'] = step+1
            performance['time'] = local_time
            performance['reward'] = reward
            performance['arrived'] = 1 if arrive_pos else 0
            performance['collision'] = 1 if col else 0
            episode_set.append(performance)
                        
            
            print('Episode %3d : steps = %3d, reward = %.2f  ' % (episode + 1, step+1, reward), end='')
            if col:          print(' >> Collision Occured.')
            elif arrive_pos: print(' >> Arrived Goal.')
            else:            print(' >> Over step.')
        
        ## Save
        if Args['eval_save']:
            
            model_name = Args['eval_file'].split('/')
            save_file = model_name[0] + '_' + Args['scene']
            save_dict_to_csv(Args['eval_path'], save_file, episode_set)
            
    
    ## ===================================
    ##             Conclusion
    ## ===================================
    envs.del_all()
    envs.sim_pause()
    
    using_time = time.time() - using_time
    reward_count /= episode
    local_step /= episode
    local_time /= episode
    global_time /= episode
    
    print("\n-----------------------------------------\n  Conclusion\n-----------------------------------------")
    print(Args['eval_file'])
    print('Scene : ', Args['scene'])
    print('Max action = (%.2f, %.2f). %s Backward.' % (max_action[0], max_action[1], ('Enable' if Args['back_enable'] else 'Disable')))
    print(f'Total execution time = {using_time}.')
    print(f'Average global planning time = {global_time}.')
    print(f'Average local planning time = {local_time}.')
    print(f'Average step = {local_step}.')
    print(f'Average reward = {reward_count}.')
    print(f'count of arrive goal = {done_count}/{episode}.')


if __name__ == "__main__":

    rospy.init_node('rrt_rl_dynamic_node')

    if Args['real']:
        print("\n=========================================\n  Real Mode\n=========================================")
        real()
    elif Args['train']:
        print("\n=========================================\n  Training Mode\n=========================================")
        train()
    else:
        print("\n=========================================\n  Evaluation Mode\n=========================================")
        eval()