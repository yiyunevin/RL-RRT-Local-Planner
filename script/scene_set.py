"""
Last Edited by Evin Hsu 2022.07.28
---------------------------------------
Setting of the simulation worlds
"""

GazeboDict = {
    ## model name
    'robot_name'   : 'youbot',
    'base_name'    : 'base',
    'dynamic_name' : 'dynamic_',
    'static_name'  : 'static_',
    ## file directory / path
    'dir_world' : '/home/user/rrt_rl_review_ws/src/rrt_rl_nav/worlds',
    'dir_model' : '/home/user/rrt_rl_review_ws/src/rrt_rl_nav/models',
    'dir_map'   : '/home/user/rrt_rl_review_ws/src/rrt_rl_nav/maps'
}



## ===========================================
##  Robot State
## ===========================================

RobotDict ={
    'youbot':
        {
            'name': 'youbot',
            'real_angle': 240,
            'max_range': 6.0,
            'min_range': 0.1,
        },
    'ttb_like_youbot':
        {
            'name': 'ttb_like_youbot',
            'real_angle': 240,
            'max_range': 3.5,
            'min_range': 0.09,
        },
    'ttb':
        {
            'name': 'ttb',
            'real_angle': 360,
            'max_range': 3.5,
            'min_range': 0.09,
        }
}



## ===========================================
##  Scene Setting
## ===========================================

## Max Step
## -------------
##  > [ total_step(arbitrary), max_step_per_episode ]
StepDict = {
    'default' : [[ -1, 500 ], [ 50000, 500]],
    'dynamic_7_4_2': [ [-1, 350], [40000, 350] ],
    'dynamic_7_4_3': [ [-1, 350], [40000, 150] ],
}


##  Object Setting
## -------------------------------------------------------------
##  > Both Simulation and Real
## -------------------------------------------------------------
##  > base         = Base model name (File at Gazebo['dir_world'] / Gazebo['dir_map'])
##  > dynamic_dict = Dynamic objects (File at Gazebo['dir_model'])
##  > robot_pos    = Robot initial position (protect from overturning in simulation world)  [ x, y, yaw ]
##  > fixed_yaw    = Fixed initial orientation or not
##  > start_goal_region_range = Specify start and goal area or not
##  > start_region = [ x_min, x_max, y_min, y_max ]
##  > goal_region  = [ x_min, x_max, y_min, y_max ]

SceneDict = {
    'dynamic_7_4_1': \
        {
            'base': 'space_4_7_1',
            'dynamic_dict' : \
                {
                    1 : {
                        'model'     : 'object_can_03',
                        'waypoints' : [ [ -1.05 ,  1.15 , 0.5 , 2.0 ],
                                        [ -1.05 , -1.15 , 0.5 , 2.0 ] ],
                        'reverse'   : True,
                    },
                    2 : {
                        'model'     : 'object_can_03',
                        'waypoints' : [ [  1.05 , -1.15 , 0.5 , 2.5 ],
                                        [  1.05  , 1.15 , 0.5 , 2.5 ] ],
                        'reverse'   : True,
                    }
                },
            'static_dict' : \
                {
                    # Empty, if static_dict == False
                }, 
            'robot_pos' : [ -2.25, 0.0, 0.0],
            'fixed_yaw' : True, 
            'start_goal_region_range' : True,
            'start_region' : [-3, -2.3, -1.15, 1.15],
            'goal_region'  : [ 1,  2, -1.15, 1.15],
        },
    'dynamic_7_4_2': \
        {
            'base': 'space_4_7_1',
            'dynamic_dict' : \
                {
                    1 : {
                        'model'     : 'object_can_03',
                        'waypoints' : [ [ -0.5,   1.25 , 0.5 , 2.0 ],
                                        [ -0.5 , -1.25 , 0.5 , 2.0 ] ],
                        'reverse'   : True,
                    },
                    2 : {
                        'model'     : 'object_can_03',
                        'waypoints' : [ [  0.5 , -1.25 , 0.5 , 1.5 ],
                                        [  1.5  , 1.25 , 0.5 , 1.5 ] ],
                        'reverse'   : True,
                    }
                },
            'static_dict' : \
                {
                    # Empty, if static_dict == False
                }, 
            'robot_pos' : [ -2.25, 0.0, 0.0],
            'fixed_yaw' : True, 
            'start_goal_region_range' : True,
            'start_region' : [-2.6, -2.6, -1.15, 1.15],
            'goal_region'  : [ 2.6,  2.6, -1.15, 1.15],
        },
    'dynamic_7_4_3': \
        {
            'base': 'space_4_7_1',
            'dynamic_dict' : \
                {
                    1 : {
                        'model'     : 'object_can_03',
                        'waypoints' : [ [ -1,   1.25 , 0.5 , 2.0 ],
                                        [ -1 , -1.25 , 0.5 , 2.0 ] ],
                        'reverse'   : True,
                    },
                    2 : {
                        'model'     : 'object_can_03',
                        'waypoints' : [ [  0   , 0 , 0.5 , 1.5 ],
                                        [  1.5 , 0 , 0.5 , 1.0 ] ],
                        'reverse'   : True,
                    }
                },
            'static_dict' : \
                {
                    # Empty, if static_dict == False
                }, 
            'robot_pos' : [ -2.25, 0.0, 0.0],
            'fixed_yaw' : True, 
            'start_goal_region_range' : True,
            'start_region' : [-2.6, -2.6, -1.15, 1.15],
            'goal_region'  : [ 2.6,  2.6, -1.15, 1.15],
        },
    'loop_3_3_1': \
        {
            'base': 'loop_3_3',
            'dynamic_dict' : \
                {
                    1 : {
                        'model'     : 'object_can_015',
                        'waypoints' : [ [ 2.6, 0.0, 0.5, 1.0 ],
                                        [ 1.3, 0.0, 0.5, 1.0 ] ],
                        'reverse'   : True,
                    },
                    2 : {
                        'model'     : 'object_can_015',
                        'waypoints' : [ [ -1, 1.3, 0.5, 1.0 ],
                                        [ -1, 2.6, 0.5, 1.0 ] ],
                        'reverse'   : True,
                    },
                    3 : {
                        'model'     : 'object_can_015',
                        'waypoints' : [ [ 0.8, 1.7, 0.5, 1.0 ],
                                        [ 0.8, 2.6, 0.5, 1.0 ],
                                        [ 0.8, 1.4, 0.5, 1.0 ]],
                        'reverse'   : False,
                    },
                },
            'static_dict' : 
                {
                    # Empty, if static_dict == False
                }, 
            'robot_pos' : [ 1, -2, 0.0],
            'fixed_yaw' : True, 
            'start_goal_region_range' : True,
            'start_region' : [ 1, 1, -2, -2],
            'goal_region'  : [ -0.7, -0.7, -2, -2],
        },

    # Add New Scene
}
