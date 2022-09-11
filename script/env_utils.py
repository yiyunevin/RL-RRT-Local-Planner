"""
Last Edited by Evin Hsu 2022.07.28
-------------------------------------
Functions for both real and simulation world
"""

#!/usr/bin/env python3
import os
import math
import numpy as np
import csv
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from config_set import max_action



## =================================
##  Action -> Vel Command 
## =================================

def action_adjust(action, back=False):
    if back:    
        action[0] = action[0] * 0.6 
        action[1] = (action[1]  -0.05) * 2
    else:
        action[0] = 0.7 * (action[0]+ max_action[0])/2
        action[1] = action[1]
    return action



## =================================
##  R_step count
## =================================

def every_step_reward(laser, goal, local_goal, action, map_size):
    w_occ = 2 
    w_lcg = 2 
    w_vel = 0.4 
    k_v = 1
    k_w = 1 
    
    ## 01 | Laser / Object Dist
    r_occ = -( 0.35 - abs(min(laser)) )/0.35  if min(laser) < 0.35 else 0
    
    ## 02 | Goal Dist
    r_lcg = - 1 * goal / (1 * map_size)
    
    ## 03 | Last Velocity
    r_vel = action[0]/k_v - abs(action[1])/k_w
    
    return w_occ*r_occ + w_lcg*r_lcg + w_vel*r_vel



## ==========================================
##  Laser Data Processing
## ==========================================
##  > MAX_ANGLE = 240.0  degree 
##  > MIN_ANGLE = 0.0    degree

## Step 1
def lidar_scan(scan_msg, robot, scan_angle=240, real=True):
    distances = []
    angles = []
    range_msg = scan_msg.ranges 
    
    ## Angle
    cut = math.floor(scan_angle/2)
    cut_end = cut + (robot['real_angle']-scan_angle)
    if robot['name'] == 'ttb': 
        range_msg = range_msg[cut_end:] + range_msg[:cut]
    else:
        range_msg = range_msg[:cut] + range_msg[cut_end:]
    range_msg = [i for i in range_msg] 
    
    ## Data
    for i, range in enumerate(range_msg):
        ang = i * scan_msg.angle_increment
        if range > robot['max_range']:
            dist = robot['max_range']
        elif range < robot['min_range']:
            dist = robot['max_range']/2 if real else robot['min_range']
        else:
            dist = range
        distances.append(dist)
        angles.append(ang)
    distances = np.array(distances)
    angles = np.array(angles)
    return distances, angles  # [m], [rad]

## Step 2
def laser_binning(data, group):
    interval = round(data.shape[0]/group)
    q_laser = []
    for i in range(group):
        lower = i * interval
        upper = min(data.shape[0], (i+1)*interval-1)
        q_laser.append(np.min(data[lower:upper]))
    return np.array(q_laser)



## ==========================================
##  Collision Check
## ==========================================

def check_collision(scan_range, collision_dist):
    col = False
    if np.min(scan_range) <= collision_dist:
        # print('Collision Occured.')
        col = True
    return col



## ==========================================
##  Odometry Processing 
## ==========================================

## position、roll(restart)、yaw(orientation)
def get_robot_position(odom_msg):
    pos = odom_msg.pose.pose.position
    p = Point()
    p.x = pos.x
    p.y = pos.y
    q = odom_msg.pose.pose.orientation
    (roll, _, yaw) = euler_from_quaternion([ q.x, q.y, q.z, q.w ])
    return p, roll, yaw



## ==========================================
##  Robot Control
## ==========================================

def robot_stop(pub):
    twist = get_twist_msg(0.0, 0.0)
    pub.publish(twist)

def robot_vel_cmd(pub, v, w):
    twist = get_twist_msg(v, w)
    pub.publish(twist)

def get_twist_msg(v, w):
    twist = Twist()
    twist.linear.x  = v
    twist.linear.y  = 0
    twist.linear.z  = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = w
    return twist

## ==========================================
##  Math Tools
## ==========================================

def euclidean2D(ax, ay, bx, by):
    return math.hypot( (bx-ax), (by-ay) )

def angle_range(ang):
    n = abs(ang) // (2*math.pi)
    return ang - np.sign(ang) * 2 * n * math.pi

def random_generate(min, max):
    return min + np.random.rand()*(max-min)

## xyzw -> yaw 
def get_yaw(q):
    (_, _, yaw) = euler_from_quaternion([ q.x, q.y, q.z, q.w ])
    return yaw


## ==============================================
##  File Saving
## ==============================================

## dictionary --> csv
def save_dict_to_csv(dir, file, dict_list):
    file = file + '.csv'
    save_path = os.path.join(dir, file) 
    label = list(dict_list[0].keys())
    try:
        with open(save_path, 'w') as f:
            writer = csv.DictWriter(f, fieldnames=label)
            writer.writeheader()
            for elem in dict_list:
                writer.writerow(elem)    
        print(f'\nSave CSV file at:\n{save_path}')
    except IOError as e:
        print('CSV saving failed.')