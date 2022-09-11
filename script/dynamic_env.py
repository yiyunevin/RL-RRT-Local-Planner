"""
Last Edited by Evin Hsu 2022.07.28
----------------------------------------
RL Environment for simulation
"""

#!/usr/bin/env python3
import os
import math
import numpy as np
import subprocess
import rospy
from geometry_msgs.msg import Point, Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, DeleteModel
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty
from config_set import EnvArgs
from scene_set import GazeboDict, RobotDict, SceneDict
from dynamic_object import DynamicObj
from env_utils import *
from rrt_rl_navigation.srv import *


class DynamicEnv:
    
    ## =====================================================
    ##  Initialization
    ## =====================================================
    
    def __init__(self, scene, robot, back=False, train=False):
        
        self.debug = EnvArgs['debug']
        self.train = train
        self.back_enable = back
        self.set_init_yaw = EnvArgs['set_init_yaw']
        self.scene = SceneDict[scene]
        SceneDict.clear()
        self.robot = RobotDict[robot]

        if self.train:
            self.goal_tol = EnvArgs['goal_dist_tol'] 
            self.goal_ang = EnvArgs['goal_angle_tol'] 
            self.inte_tol = EnvArgs['inter_point_tol'] 
            self.coll_tol = EnvArgs['collision_dist']
        else:
            self.goal_tol = EnvArgs['eval_goal_tol']
            self.goal_ang = EnvArgs['eval_angle_tol']
            self.inte_tol = EnvArgs['eval_inter_tol']
            self.coll_tol = EnvArgs['eval_coll_dist']
            
        ## ROS Topic Publisher / Service Client
        self.base_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        
        ## objects
        self.dynamic_list = []
        self.dynamic_object = []
        
        ## Rviz marker
        self.mk_start_pub = rospy.Publisher('/marker_start', Marker, queue_size=10)
        self.mk_goal_pub = rospy.Publisher('/marker_goal', Marker, queue_size=10)
        self.mk_inter_pub = rospy.Publisher('/marker_inter', Marker, queue_size=20)
        self.mk_start = Marker()
        self.mk_goal = Marker()
        self.mk_inter = Marker()
        self.marker_reset()
        
        ## robot stop
        self.robot_cmd(0, 0)
    
    
    ## destructure
    def del_all(self):
        self.sim_unpause()
        for dd in self.dynamic_list:
            self.delete_model(dd)
        self.delete_world()
        self.sim_pause()
    
    
    ## =================================
    ## RViz Marker
    ## =================================
    
    def marker_tree_init(self, seq):
        mk = Marker()
        mk.id = seq
        mk.header.frame_id = 'odom'
        mk.header.stamp = rospy.get_rostime()
        mk.ns = 'markers'
        mk.action = Marker.ADD
        mk.scale.x = 0.1
        mk.scale.y = 0.1
        mk.pose.orientation.w = 1.0
        mk.color.r = 0.
        mk.color.g = 1.
        mk.color.b = 0.
        mk.color.a = 1.
        return mk
    
    
    def marker_reset(self):
        self.mk_start.action = Marker.DELETEALL
        self.mk_start_pub.publish(self.mk_start)
        self.mk_start.points = []
        
        self.mk_goal.action = Marker.DELETEALL
        self.mk_goal_pub.publish(self.mk_goal)
        self.mk_goal.points = []
        
        self.mk_inter.action = Marker.DELETEALL
        self.mk_inter.points = []
        self.mk_inter_pub.publish(self.mk_inter)
        
        self.mk_goal = self.marker_tree_init(0)    ## Green
        self.mk_goal.type = Marker.POINTS
        
        self.mk_start = self.marker_tree_init(0)   ## Yellow
        self.mk_start.type = Marker.POINTS
        self.mk_start.color.r = 1.
        self.mk_start.color.g = 1.
        self.mk_start.color.b = 0.2
        
        self.mk_inter = self.marker_tree_init(7)   ## cyn ...
        self.mk_inter.type = Marker.POINTS
        self.mk_inter.color.r = 0.
        self.mk_inter.color.g = 1.
        self.mk_inter.color.b = 1.
    
    
    
    ## ==================================================================================
    ##  Scene Setting
    ## ==================================================================================
    
    ## Scene Initialization
    def scene_initial(self):
        
        self.sim_unpause()
        
        self.robot_cmd(0, 0)
        self.set_model_state(GazeboDict['robot_name'], self.scene['robot_pos'][0], self.scene['robot_pos'][1],  self.scene['robot_pos'][2])

        self.delete_world()
        rospy.sleep(0.5)

        path_world = os.path.join(GazeboDict['dir_world'], self.scene['base'], "model.sdf")
        subprocess.Popen(["rosrun", "gazebo_ros", "spawn_model", "-file", path_world, "-sdf", "-model", GazeboDict['base_name'], "-x", "0.0", "-y", "0.0"])
        rospy.sleep(0.5)

        path_yaml = os.path.join(GazeboDict['dir_map'], self.scene['base'], "map.yaml")
        subprocess.Popen(["rosrun", "map_server", "map_server", path_yaml])
        rospy.sleep(0.5)

        self.get_map_info()

        if self.scene['dynamic_dict']:   # if not empty
            self.dynamic_initial(self.scene['dynamic_dict'])
        
        return True
    
    def dynamic_initial(self, dynamic_dict):
        self.dynamic_list = []
        self.dynamic_object = []
        
        for key in dynamic_dict:
            nm = 'dynamic_' + str(key)  ## model name
            wp = []                     ## waypoints
            ts = []                     ## time_sec between waypoints
            for j in range(len(dynamic_dict[key]['waypoints'])):
                wp.append( [ dynamic_dict[key]['waypoints'][j][0] , dynamic_dict[key]['waypoints'][j][1] , dynamic_dict[key]['waypoints'][j][2] ] )     ## [ x, y, z ]
                ts.append( dynamic_dict[key]['waypoints'][j][3] )   ## [ ts ]
            if len(dynamic_dict[key]['waypoints']) > 1:
                wp.append(wp[0])
            self.dynamic_list.append(nm)
            self.dynamic_object.append( DynamicObj(dynamic_dict[key]['model'], nm, wp, ts, dynamic_dict[key]['reverse']) )
        
        if self.debug:
            for i in range(len(self.dynamic_list)):
                print('\n== Dynamic Object %d ==' % (i+1))
                print('Name : ', self.dynamic_object[i].model_name)
                for j in range(len(self.dynamic_object[i].time_sec)):
                    print('-> Waypoint between ( %.2lf, %.2lf, %.2lf ) and ( %.2lf, %.2lf, %.2lf ), %.2lf sec.' % \
                            (   self.dynamic_object[i].waypoints[j][0], self.dynamic_object[i].waypoints[j][1], self.dynamic_object[i].waypoints[j][2], \
                                self.dynamic_object[i].waypoints[j+1][0], self.dynamic_object[i].waypoints[j+1][1], self.dynamic_object[i].waypoints[j+1][2], self.dynamic_object[i].time_sec[j]))
                print('Loop Mode : %s' % ( 'Reverse' if  self.dynamic_object[i].reverse else 'Loop')) 
            print('')
    
    
    # ==================================================================================
    #  One Step
    # ==================================================================================    
    
    def step(self, action, init=False):
        
        col, arrived_pos = False, False
        reward = 0
        
        ## Environment Update
        if not init:
            ax = action_adjust(action, self.back_enable)
            self.robot_cmd(ax[0], ax[1])
            if EnvArgs['state_freq'] > 0:
                rospy.sleep(EnvArgs['state_freq'])
        self.sim_unpause()
        for dyo in self.dynamic_object:
            dyo.move_next_step()
        
        
        ## State Update
        l_dist, l_ang, l_ang_min = get_laser_data(EnvArgs['scan_topic'], self.robot, EnvArgs['scan_angle'], False)
        if not len(l_dist) > 0:
            if self.debug:
                print("No laser data. restart.")
            return None, None, None, None, True
        laser_data = laser_binning(l_dist, EnvArgs['laser_data_size'])

        r_pos, roll, r_ang = get_odom_data(EnvArgs['odom_topic'])
        if abs(roll) > EnvArgs['robot_wrong']:
            if self.debug:
                print("Robot has trouble. restart.")
            return None, None, None, None, True

        local_goal, rr , self.inter_point = self.update_inter_point(r_pos, r_ang, self.goal, self.inter_point)
        local_goal_all = self.get_local_goal(r_pos, r_ang, self.goal)   ## 終點相對極座標 (不一定有用)
        self.sim_pause()

        state = [e for ll in [laser_data, local_goal, action] for e in ll]
        
        arrived_pos = self.goal_is_arrived(r_pos, self.goal)
        col = self.check_collision(laser_data)
        
        ## Reward
        if not init:
            if col:
                reward = EnvArgs['collision_R']
            elif arrived_pos:
                reward = EnvArgs['arrived_R']
            else:
                map_size = abs(max(self.m_origin_x, self.m_origin_y))
                reward = every_step_reward(laser_data, local_goal_all[0], None, ax, map_size)
                if EnvArgs['state_class'] == 'lg':
                    reward = EnvArgs['arrived_R'] if rr > 0 else reward
                if EnvArgs['state_class'] == 'glg':
                    reward = EnvArgs['inter_R'] if rr > 0 else reward
        
        return state, col, arrived_pos, reward, False
    
    def goal_is_arrived(self, r_pos, goal_pos):
        return True if euclidean2D(r_pos.x, r_pos.y, goal_pos.x, goal_pos.y) <= self.goal_tol else False
    
    def check_collision(self, scan_range):
        return True if np.min(scan_range) < self.coll_tol else False
    
    def update_inter_point(self, r_pos, r_ang, goal, inter_point):
        arrive_count = 0
        if len(inter_point) > 0 and euclidean2D(r_pos.x, r_pos.y, inter_point[0].x, inter_point[0].y) <= self.inte_tol:
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
    
    ## ========================================================================
    ##  Global Planning
    ## ========================================================================
    def get_global_path(self):
        if EnvArgs['use_global']:
            if self.scene['start_goal_region_range']:
                ret = self.get_global_start_goal()
            else:
                ret = self.get_global_random()
            if not ret: 
                return False
            
            if self.debug:
                print(f'\nGenerate Global Path from ({self.start.x}, {self.start.y}) to ({self.goal.x}, {self.goal.y}). \n')
            
            if self.scene['fixed_yaw']:
                self.start_ang = self.scene['robot_pos'][2]
            elif self.set_init_yaw:
                if len(self.inter_point) > 0:
                    self.start_ang = math.atan2((self.inter_point[0].y - self.start.y), (self.inter_point[0].x - self.start.x))
                else:
                    self.start_ang = math.atan2((self.goal.y - self.start.y), (self.goal.x - self.start.x))
        else:
            ret = self.get_start_goal()  # No inter_point
            if not ret:
                return False
            if self.scene['fixed_yaw']:
                self.start_ang = self.scene['robot_pos'][2]
            elif self.set_init_yaw:
                self.start_ang = math.atan2((self.goal.y - self.start.y), (self.goal.x - self.start.x))
        
        ## Marker
        p = Point()
        p.x = self.goal.x
        p.y = self.goal.y
        p.z = 0.02
        self.mk_goal.points.append(p)
        self.mk_goal_pub.publish(self.mk_goal)
        self.mk_goal.points = []
        p.x = self.start.x
        p.y = self.start.y
        self.mk_start.points.append(p)
        self.mk_start_pub.publish(self.mk_start)
        self.mk_start.points = []
        p.z = 0.05
        for i, b in enumerate(self.inter_point):
            p.x = b.x
            p.y = b.y
            self.mk_inter.points.append(p)
        self.mk_inter_pub.publish(self.mk_inter)
        self.mk_inter.points = []
        
        self.init_local_goal = self.get_local_goal(self.start, self.start_ang, self.goal)
        self.set_model_state(GazeboDict['robot_name'], self.start.x, self.start.y, self.start_ang)
        self.robot_cmd(0, 0)
        rospy.sleep(1)
        
        return True
    
    def get_map_info(self) -> None:
        rospy.wait_for_service('occupancy_map_info')
        try:
            client = rospy.ServiceProxy('occupancy_map_info', OccMapInfo)
            res = client.call()
            self.m_resolution = res.resolution
            self.m_width = res.width
            self.m_height = res.height
            self.m_origin_x = res.origin_x
            self.m_origin_y = res.origin_y
            
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)
    
    
    def get_start_goal(self):
        q = quaternion_from_euler(0., 0., self.scene['robot_pos'][2])
        start = PoseStamped()
        start.pose.position.x = random_generate(self.scene['start_region'][0], self.scene['start_region'][1])
        start.pose.position.y = random_generate(self.scene['start_region'][2], self.scene['start_region'][3])
        start.pose.orientation.x = q[0]
        start.pose.orientation.y = q[1]
        start.pose.orientation.z = q[2]
        start.pose.orientation.w = q[3]
        goal  = PoseStamped()
        goal.pose.position.x = random_generate(self.scene['goal_region'][0], self.scene['goal_region'][1])
        goal.pose.position.y = random_generate(self.scene['goal_region'][2], self.scene['goal_region'][3])
        goal.pose.orientation = start.pose.orientation
        
        # if np.random.uniform(0, 1) > 0.5:
        #     temp = start
        #     start = goal
        #     goal = temp
        
        self.start = Point()
        self.start.x = start.pose.position.x
        self.start.y = start.pose.position.y
        self.start_ang = self.scene['robot_pos'][2]
        self.goal = Point()
        self.goal.x = goal.pose.position.x
        self.goal.y = goal.pose.position.y
        self.goal_ang = self.scene['robot_pos'][2]
        self.inter_point = []
        
        if self.debug:
            print('\nGenerate Start & Goal from (%.2f, %.2f) to (%.2f, %.2f). \n' % \
                    (start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y))
        return True
    
    
    def get_global_random(self):
        self.start = Point()
        self.goal = Point()
        self.inter_point = []
        rospy.wait_for_service('random_path_inter')
        try:
            client = rospy.ServiceProxy('random_path_inter', RandomGetPath)
            
            res = client.call()
            if res.success:
                
                self.start.x = res.start.pose.position.x
                self.start.y = res.start.pose.position.y
                self.start_ang = get_yaw(res.start.pose.orientation)
                self.goal.x = res.goal.pose.position.x
                self.goal.y = res.goal.pose.position.y
                self.goal_ang = get_yaw(res.goal.pose.orientation)
                for bb in res.inters:
                    inter = Point()
                    inter.x = bb.pose.position.x
                    inter.y = bb.pose.position.y
                    self.inter_point.append(inter)
                if len(self.inter_point) > 0:
                    del self.inter_point[-1]
                
                # if np.random.uniform(0, 1) > 0.5:
                #     temp = self.start
                #     self.start = self.goal
                #     self.goal = temp
                    
                #     temp = self.start_ang
                #     self.start_ang = self.goal_ang
                #     self.goal_ang = temp
                    
                #     self.inter_point.reverse()
                
                if self.debug:
                    print("\nGlobal Planning Successful!\nfrom (%lf, %lf) to (%lf, %lf)" % (self.start.x, self.start.y, self.goal.x, self.goal.y))
            else:
                return False
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e) 
        return True
    
    
    # Get Path with given start and goal
    def get_global_start_goal(self):
        q = quaternion_from_euler(0., 0., self.scene['robot_pos'][2])
        start = PoseStamped()
        start.pose.position.x = random_generate(self.scene['start_region'][0], self.scene['start_region'][1])
        start.pose.position.y = random_generate(self.scene['start_region'][2], self.scene['start_region'][3])
        start.pose.orientation.x = q[0]
        start.pose.orientation.y = q[1]
        start.pose.orientation.z = q[2]
        start.pose.orientation.w = q[3]
        goal  = PoseStamped()
        goal.pose.position.x = random_generate(self.scene['goal_region'][0], self.scene['goal_region'][1])
        goal.pose.position.y = random_generate(self.scene['goal_region'][2], self.scene['goal_region'][3])
        goal.pose.orientation = start.pose.orientation
        
        if np.random.uniform(0, 1) > 0.5:
            temp  = start
            start = goal
            goal  = temp
        
        self.start = Point()
        self.start.x = start.pose.position.x
        self.start.y = start.pose.position.y
        self.start_ang = self.scene['robot_pos'][2]
        self.goal = Point()
        self.goal.x = goal.pose.position.x
        self.goal.y = goal.pose.position.y
        self.goal_ang = self.scene['robot_pos'][2]
        self.inter_point = []

        rospy.wait_for_service('start_goal_path_inter')   # from global_planner
        try:
            client = rospy.ServiceProxy('start_goal_path_inter', StartGoalPath)
            res = client.call(start, goal)
            if res.success:
                self.start_ang = get_yaw(start.pose.orientation)
                self.goal_ang = get_yaw(goal.pose.orientation)
                for bb in res.inters:
                    inter = Point()
                    inter.x = bb.pose.position.x
                    inter.y = bb.pose.position.y
                    self.inter_point.append(inter) 
            else:
                return False
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e) 
        return True
    
    
    ## ===============================
    ##  Gazebo Control
    ## ===============================
    
    def robot_cmd(self, v, w):
        twist = Twist()
        twist.linear.x  = v
        twist.linear.y  = 0
        twist.linear.z  = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = w
        self.sim_unpause()
        self.base_vel_pub.publish(twist)
        self.sim_unpause()
    
    def sim_pause(self):
        for i in range(len(self.dynamic_object)):
            self.dynamic_object[i].deactivate()
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            pass 
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed.")
    
    def sim_unpause(self):
        for i in range(len(self.dynamic_object)):
            self.dynamic_object[i].activate()
            
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed.")    
    
    def set_model_state(self, name, x, y, ang):
        q = quaternion_from_euler(0., 0., ang)
        state = ModelState()
        state.model_name = name
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.orientation.x = q[0]
        state.pose.orientation.y = q[1]
        state.pose.orientation.z = q[2]
        state.pose.orientation.w = q[3]
        self.sim_unpause()
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            res = client.call(state)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e) 
    
    def delete_model(self, model_name):
        rospy.wait_for_service('/gazebo/delete_model')
        try:
            client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            res = client.call(model_name)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e) 
    
    def delete_world(self):
        model_name = "\'" + GazeboDict['base_name'] + "\'"
        subprocess.Popen(["rosservice", "call", "/gazebo/delete_model", model_name])


def get_laser_data(topic, robot, scan_angle, real):
    scan_msg = None
    while scan_msg is None:
        try:
            scan_msg = rospy.wait_for_message(topic, LaserScan, timeout=0.5)
        except:
            pass
    l_ang_min = scan_msg.angle_min
    l_dist, l_ang = lidar_scan(scan_msg, robot, scan_angle, real)
    return l_dist, l_ang, l_ang_min

def get_odom_data(topic):
    odom_msg = None
    while odom_msg is None:
        try:
            odom_msg = rospy.wait_for_message(topic, Odometry, timeout=0.5)
        except:
            pass
    return get_robot_position(odom_msg)