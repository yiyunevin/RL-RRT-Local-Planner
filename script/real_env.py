"""
Last Edited by Evin Hsu 2022.07.28
-------------------------------------
RL Environment for Real World
"""

#!/usr/bin/env python3
import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from config_set import EnvArgs
from scene_set import RobotDict
from rrt_planner_client import GlobalClient
from env_utils import *
from rrt_rl_navigation.srv import *

class RealEnv:

## ==================================================================================
##   Initialization
## ==================================================================================

    def __init__(self, robot, back_enable=False) -> None:

        self.debug = EnvArgs['debug']
        self.back_enable = back_enable
        
        self.goal_tol = EnvArgs['eval_goal_tol']
        self.goal_ang = EnvArgs['eval_angle_tol']
        self.inte_tol = EnvArgs['eval_inter_tol']
        self.coll_tol = EnvArgs['eval_coll_dist']
        
        self.robot = RobotDict[robot]
        
        self.global_cl = GlobalClient()

        ## Flag for Main
        self.global_plan = False
        self.local_plan = False
        self.done = True 

        self.scan_topic = "/scan"
        self.odom_topic = "/odom"

        ret, self.m_resolution, self.m_width, self.m_height, self.m_origin_x, self.m_origin_y = self.global_cl.get_map_info()
        while not ret:
            ret, self.m_resolution, self.m_width, self.m_height, self.m_origin_x, self.m_origin_y = self.global_cl.get_map_info()

        self.base_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.CB_rviz_goal)
        rospy.Subscriber(self.scan_topic, LaserScan, self.CB_laser)
        rospy.Subscriber(self.odom_topic, Odometry, self.CB_odom)

        self.goal = None 
        self.laser_data = None 
        self.odom_data  = None

        self.robot_cmd(0, 0)


# ====================================
#   Callback
# ====================================

    ## Get goal information from Rviz 2D Nav Goal

    def CB_rviz_goal(self, data):
        self.robot_cmd(0, 0)
        self.goal_stamp = data
        self.start_stamp = self.get_start()     ## start = current robot position
        print('>> Generate Global Path from (%.2lf, %.2lf) to (%.2lf, %.2lf). \n' % 
                (self.start_stamp.pose.position.x, self.start_stamp.pose.position.y, self.goal_stamp.pose.position.x, self.goal_stamp.pose.position.y))
        self.local_plan = False
        self.global_plan = True
    
    def CB_laser(self, data):
        self.laser_data = data

    def CB_odom(self, data):
        if not self.odom_data == data:
            self.odom_data = data

# ======================================
#   One Step
# ======================================
    
    ## RL Step
    def step(self, action, init=False):
        
        col, arrived_pos = False, False 
        reward = 0

        if not init:
            ax = action_adjust(action, self.back_enable)
            self.robot_cmd(ax[0], ax[1])
            if EnvArgs['state_freq'] > 0:
                rospy.sleep(EnvArgs['state_freq'])

        l_dist, _, _ = self.get_laser_data(self.scan_topic, self.robot, EnvArgs['scan_angle'], True)
        if not len(l_dist) > 0:
            if self.debug:
                print("No laser data. restart.")
            return None, None, None, None, True

        r_pos, _, r_ang = self.get_odom_data(self.odom_topic)
        

        laser_data = laser_binning(l_dist, EnvArgs['laser_data_size'])
        local_goal, rr , self.inter_point = self.update_inter_point(r_pos, r_ang, self.goal, self.inter_point)
        state = [e for ll in [laser_data, local_goal, action] for e in ll]

        arrived_pos = self.goal_is_arrived(r_pos, self.goal)
        col = self.check_collision(laser_data)
        
        if col or arrived_pos: print()

        if not init:
            if col:            
                reward = EnvArgs['collision_R']
            elif arrived_pos:  
                reward = EnvArgs['arrived_R']
            elif rr > 0:       
                reward = EnvArgs['arrived_R']
            else:              _origin_y))
                reward = every_step_reward(laser_data, local_goal[0], None, ax, map_size)
        
        return state, col, arrived_pos, reward, False

    def get_laser_data(self, topic, robot, scan_angle, real):
        scan_msg = None
        while scan_msg is None:
            try:
                scan_msg = rospy.wait_for_message(topic, LaserScan, timeout=0.5)
            except:
                pass
        l_ang_min = scan_msg.angle_min
        l_dist, l_ang = lidar_scan(scan_msg, robot, scan_angle, real)
        return l_dist, l_ang, l_ang_min

    def get_odom_data(self, topic):
        odom_msg = None
        while odom_msg is None:
            try:
                odom_msg = rospy.wait_for_message(topic, Odometry, timeout=0.5)
            except:
                pass
        return get_robot_position(odom_msg)  # control

    def robot_cmd(self, v, w):
        twist = Twist()
        twist.linear.x  = v
        twist.linear.y  = 0
        twist.linear.z  = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = w
        self.base_vel_pub.publish(twist)

    def goal_is_arrived(self, r_pos, goal_pos):
        print('Arr : %.4lf; ' % euclidean2D(r_pos.x, r_pos.y, goal_pos.x, goal_pos.y), end='')
        return True if euclidean2D(r_pos.x, r_pos.y, goal_pos.x, goal_pos.y) <= self.goal_tol else False

    def check_collision(self, scan_range):
        print('Coll : %.4lf' % np.min(scan_range))
        return True if np.min(scan_range) < self.coll_tol else False

    def update_inter_point(self, r_pos, r_ang, goal, inter_point):
        arrive_count = 0
        if len(inter_point) > 0 and euclidean2D(r_pos.x, r_pos.y, inter_point[0].x, inter_point[0].y) <= self.inte_tol:
            print('Arrive Intermidiate Point at (%.2lf, %.2lf)' % (inter_point[0].x, inter_point[0].y))
            arrive_count = 1
            del inter_point[0]
        if len(inter_point) > 0:
            lg = self.get_local_goal(r_pos, r_ang, inter_point[0])
        else:
            lg = self.get_local_goal(r_pos, r_ang, goal)
        return lg, arrive_count, inter_point

    def get_local_goal(self, r_pos, r_ang, g_pos):
        r = euclidean2D(r_pos.x, r_pos.y, g_pos.x, g_pos.y)
        if r > 0:
            ang = math.atan2((g_pos.y-r_pos.y), (g_pos.x-r_pos.x))
            ang = ang - r_ang
            if ang > np.pi:
                ang = -2 * np.pi + ang
            if ang < -np.pi:
                ang = 2 * np.pi + ang
            return [r, ang]
        else:
            return [0, 0]
    
# ==================================================================================
#  Global Planning
# ================================================================================== 

    def get_start(self):
        odom_msg = None
        while odom_msg is None:
            try:
                odom_msg = rospy.wait_for_message(self.odom_topic, Odometry, timeout=0.5)
            except:
                pass
        start = PoseStamped()
        start.header = odom_msg.header
        start.pose = odom_msg.pose.pose        
        return start

    def get_goal_rviz(self):
        return self.goal
        
    ##  (in rrt_planner_client.py)
    def get_global_path(self):
        self.start = Point()
        self.goal = Point()
        self.inter_point = []
        if EnvArgs['use_global']:
            ret, self.goal_stamp, self.inter_point = self.global_cl.get_global_path(self.start_stamp, self.goal_stamp)
        else:
            ret = True
        if ret:
            self.start.x = self.start_stamp.pose.position.x
            self.start.y = self.start_stamp.pose.position.y
            self.start_ang = get_yaw(self.start_stamp.pose.orientation)
            self.goal.x = self.goal_stamp.pose.position.x
            self.goal.y = self.goal_stamp.pose.position.y
            self.goal_ang = get_yaw(self.goal_stamp.pose.orientation)
            if len(self.inter_point) > 0:
                self.init_local_goal = self.get_local_goal(self.start, self.start_ang, self.inter_point[0])
            else:
                self.init_local_goal = self.get_local_goal(self.start, self.start_ang, self.goal)
            self.robot_cmd(0, 0)
            rospy.sleep(1)
            return True
        else:
            return False