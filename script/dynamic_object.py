"""
Last Edited by Evin Hsu 2022.07.28
---------------------------------------
Dynamic Object
"""

import subprocess
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose
from env_utils import *
from scene_set import GazeboDict

class DynamicObj:

## =====================================
##  Initialization
## =====================================
    
    def __init__(self, model_type, model_name, waypoints, time_sec, reverse=False, time_resolution=0.02):
    
        self.model_type = model_type            # Model Type                       ( String )
        self.model_name = model_name            # Name of Model                    ( String )
        self.waypoints = waypoints              # Waypoints                        ( List of list [x, y, z] )
        self.time_sec = time_sec                # Step between waypoints           ( List of Double )
        self.reverse = reverse                  # Loop mode --> True : Reverse , False : circular    ( Boolean )
        
        self.step_count = 0 
        self.time_resolution = time_resolution  # Waypoints resolution
        self.active = True                      # movable or not
        self.get_every_step()                   # Normalize step
        self.spawn_model(model_type, model_name, waypoints[0])
    
    
    def __del__(self):
        model_name = "\'" + self.model_name + "\'"
        subprocess.Popen(["rosservice", "call", "/gazebo/delete_model", model_name])
    
    
    def spawn_model(self, model_type, model_name, position):
        path = os.path.join(GazeboDict['dir_model'], model_type, "model.sdf")
        subprocess.Popen(["rosrun", "gazebo_ros", "spawn_model", "-file", path, "-sdf", "-model", model_name, "-x", str(position[0]), "-y", str(position[1])])
    
    def get_every_step(self):
        self.every_steps = []
        for i, ts in enumerate(self.time_sec):
            theta = math.atan2( self.waypoints[i+1][1] - self.waypoints[i][1], \
                                self.waypoints[i+1][0] - self.waypoints[i][0] )
            move_dist = euclidean2D(  self.waypoints[i][0],   self.waypoints[i][1], \
                                            self.waypoints[i+1][0], self.waypoints[i+1][1])
            time_step = math.floor( ts/self.time_resolution )
            move_step = move_dist/float(time_step)
            z_step = ( self.waypoints[i+1][2] - self.waypoints[i][2] )/float(time_step)
        
            for j in range(time_step):
                pose = Pose()
                self.every_steps.append(pose)
                self.every_steps[-1].position.x = self.waypoints[i][0] + j * move_step * math.cos(theta)
                self.every_steps[-1].position.y = self.waypoints[i][1] + j * move_step * math.sin(theta)
                self.every_steps[-1].position.z = self.waypoints[i][2] + j * z_step
    
    
## =====================================
##  Move Every Step
## =====================================

    def get_next_step(self):
        if self.active:
            if self.step_count == len(self.every_steps):
                if self.reverse == True:
                    self.every_steps.reverse()
                self.step_count = 1
            pose = self.every_steps[self.step_count]
            self.step_count = self.step_count + 1
        else:
            if self.step_count == 0:
                pose = self.every_steps[0]
            else:
                pose = self.every_steps[self.step_count-1]
        return pose

    def move_next_step(self):
        model_state_msg = ModelState()
        model_state_msg.model_name = self.model_name
        model_state_msg.reference_frame = "world"
        model_state_msg.pose = self.get_next_step()
        
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_pub = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
            resp = set_model_pub(model_state_msg)
        except:
            print("Dynamic object failed to move to next step.")


## =====================================
##  Pause / Unpause
## =====================================

    def activate(self):
        self.active = True

    def deactivate(self):
        self.active = False