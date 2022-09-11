"""
Last Edited by Evin Hsu 2022.07.28
---------------------------------------
Bridge with the Global Planner
"""

#!/usr/bin/env python3
import numpy as np
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from rrt_planner.srv import *

def random_generate(min, max):
    return min + np.random.rand()*(max-min)


class GlobalClient:

## ==================================================================================
##  Initialization
## ==================================================================================

    def __init__(self) -> None:

        self.mk_start_pub = rospy.Publisher('/marker_start', Marker, queue_size=1)
        self.mk_start = Marker()
        self.mk_start_color = [1., 0., 0.]
        self.mk_goal_pub = rospy.Publisher('/marker_goal', Marker, queue_size=1)
        self.mk_goal = Marker()
        self.mk_goal_color = [0., 1., 0.]
        self.mk_inter_pub = rospy.Publisher('/marker_inter', Marker, queue_size=10)
        self.mk_inter = Marker()
        self.mk_inter_color = [0., 1., 1.]
        self.mk_path_pub = rospy.Publisher('/marker_path', Marker, queue_size=10)
        self.mk_path = Marker()
        self.mk_path_color = [0., 0., 1.]
        self.marker_initial()


## ==================================================================================
##  Marker
## ==================================================================================
    
    def marker_tree_init(self, seq):
        mk = Marker()
        mk.id = seq
        mk.header.frame_id = 'map'
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
    
    def marker_initial(self):
        
        self.mk_start = self.marker_tree_init(5)   ## Green
        self.mk_start.type = Marker.POINTS
        self.mk_start.color.r = self.mk_start_color[0]
        self.mk_start.color.g = self.mk_start_color[1]
        self.mk_start.color.b = self.mk_start_color[2]
        
        self.mk_goal = self.marker_tree_init(6)    ## Red
        self.mk_goal.type = Marker.POINTS
        self.mk_goal.color.r = self.mk_goal_color[0]
        self.mk_goal.color.g = self.mk_goal_color[1]
        self.mk_goal.color.b = self.mk_goal_color[2]
        
        self.mk_inter = self.marker_tree_init(7)   ## cyan
        self.mk_inter.type = Marker.POINTS
        self.mk_inter.color.r = self.mk_inter_color[0]
        self.mk_inter.color.g = self.mk_inter_color[1]
        self.mk_inter.color.b = self.mk_inter_color[2]
        
        self.mk_path = self.marker_tree_init(8)   ## blue
        self.mk_path.type = Marker.LINE_LIST
        self.mk_path.color.r = self.mk_path_color[0]
        self.mk_path.color.g = self.mk_path_color[1]
        self.mk_path.color.b = self.mk_path_color[2]
        
        self.marker_reset()
    
    
    def marker_reset(self):
        self.mk_start.action = Marker.DELETEALL
        self.mk_start.points = []
        self.mk_start_pub.publish(self.mk_start)
        self.mk_start.action = Marker.ADD
        
        self.mk_goal.action = Marker.DELETEALL
        self.mk_goal.points = []
        self.mk_goal_pub.publish(self.mk_goal)
        self.mk_goal.action = Marker.ADD
        
        self.mk_inter.action = Marker.DELETEALL
        self.mk_inter.points = []
        self.mk_inter_pub.publish(self.mk_inter)
        self.mk_inter.action = Marker.ADD
        
        self.mk_path.action = Marker.DELETEALL
        self.mk_path.points = []
        self.mk_path_pub.publish(self.mk_path)
        self.mk_path.action = Marker.ADD
        
    def map_to_plot(self, mx, my):
        _, res, w, h, ox, oy = self.get_map_info()
        wx = int((mx - ox)/ res)
        wy = int((my - oy)/ res)
        if wx >= h:
            wx = h - 1
        if wy >= w:
            wy = w - 1 
        return wx, wy
        


## ==================================================================================
##  Map Information
## ==================================================================================

    def get_map_info(self) :
        rospy.wait_for_service('occupancy_map_info')
        try:
            client = rospy.ServiceProxy('occupancy_map_info', OccMapInfo)
            res = client.call()
            return True, res.resolution, res.width, res.height, res.origin_x, res.origin_y
            
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)
            return False, None, None, None, None, None
    
    def get_occ_value(self, x=0., y=0.) :
        p = Point()
        p.z = 0.15
        p.x = x
        p.y = y
        self.mk_start.points.append(p)
        self.mk_start_pub.publish(self.mk_start)
        self.mk_start.points = []  
        
        rospy.wait_for_service('occupancy_value')
        try:
            client = rospy.ServiceProxy('occupancy_value', MapOcc)
            res = client.call(x, y)
            return True, res.occ
            
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)
            return False, None

## ==================================================================================
##  Path Planning
## ==================================================================================
## Start, Goal : PoseStamped

    def get_global_path(self, start=None, goal=None):
        
        ret = False
        inters = []
        self.marker_reset()
        
        print('>> Generate Global Path from (%.2lf, %.2lf) to (%.2lf, %.2lf).' % 
                (start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y))
        
        ## Marker
        p = Point()
        ## Goal
        p.z = 0.15
        p.x = goal.pose.position.x
        p.y = goal.pose.position.y
        # p.x, p.y = self.map_to_plot(goal.pose.position.x, goal.pose.position.y)
        self.mk_goal.points.append(p)
        self.mk_goal_pub.publish(self.mk_goal)
        self.mk_goal.points = []
        ## Start
        p.x = start.pose.position.x
        p.y = start.pose.position.y
        self.mk_start.points.append(p)
        self.mk_start_pub.publish(self.mk_start)
        self.mk_start.points = []  

        if not (start is None or goal is None):
            ret, inters = self.get_global_start_goal(start, goal)
        else:
            ret, start, goal, inters = self.get_global_random()
        if not ret:
            return False, None, None
        
        ## Inter & Path (bug)
        self.mk_inter.points = []
        if len(inters) > 0:
            p.z = 0.3
            for i, b in enumerate(inters):
                p.x = b.x
                p.y = b.y
                print('>> Inter %d : (%.2lf, %.2lf)' % (i+1, b.x, b.y))
                self.mk_inter.points.append(p)
                if i > 0:
                    self.mk_path.points.append(self.mk_inter.points[-1])
                    self.mk_path.points.append(self.mk_inter.points[-2])
            
            self.mk_inter_pub.publish(self.mk_inter)
            self.mk_path_pub.publish(self.mk_path)
        
        
        return True, goal, inters


    ## Get Path with given start and goal
    def get_global_start_goal(self, start, goal):
        inters = []
        rospy.wait_for_service('start_goal_path_inter')   # from global_planner
        try:
            client = rospy.ServiceProxy('start_goal_path_inter', StartGoalPath)
            res = client.call(start, goal)
            if res.success:
                for bb in res.inters:
                    inter = Point()
                    inter.x = bb.pose.position.x
                    inter.y = bb.pose.position.y
                    inters.append(inter) 
            else:
                return False, None
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e) 
        return True, inters


    def get_global_random(self):
        start = PoseStamped()
        goal = PoseStamped()
        inters = []
        rospy.wait_for_service('random_path_inter') 
        try:
            client = rospy.ServiceProxy('random_path_inter', RandomGetPath)
            res = client.call()
            if res.success:
                start = res.start
                goal = res.goal
                for bb in res.inters:
                    inter = Point()
                    inter.x = bb.pose.position.x
                    inter.y = bb.pose.position.y
                    inters.append(inter)
                
                ## Marker
                p = Point()
                ## Goal
                p.z = 0.15
                p.x = goal.pose.position.x
                p.y = goal.pose.position.y
                self.mk_goal.points.append(p)
                self.mk_goal_pub.publish(self.mk_goal)
                self.mk_goal.points = []
                ## Start
                p.x = start.pose.position.x
                p.y = start.pose.position.y
                self.mk_start.points.append(p)
                self.mk_start_pub.publish(self.mk_start)
                self.mk_start.points = []  
            else:
                return False, None, None
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e) 
        return True, start, goal, inters