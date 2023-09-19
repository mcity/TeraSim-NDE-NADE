## this launcher file only works for Linux, tested on Ubuntu 18.04, but should work on any
## for other system, you might need to write your launcher to achieve similar goal according to:
## https://stackoverflow.com/questions/19308415/execute-terminal-command-from-python-in-new-terminal-window
import asyncio
import os
import roslaunch
from roslaunch.parent import ROSLaunchParent
import time

from ascs.utils.process import CMDHandler
from ascs.utils.rostools import publish_initial_pose

import settings


class ROSLaunchParentNew(ROSLaunchParent):
    def _start_infrastructure_new(self):
        """
        load config, start XMLRPC servers and process monitor
        """
        self._load_config()

        # Start the process monitor
        self._start_pm()

        # Startup the roslaunch runner and XMLRPC server.
        # Requires pm
        if self.server is None:
            self._start_server()

        # Startup the remote infrastructure.
        # Requires config, pm, and server
        self._start_remote()

    def restart(self):
        self.logger.info("starting roslaunch parent run")
        
        # load config, start XMLRPC servers and process monitor
        try:
            self._start_infrastructure_new()
        except:
            # infrastructure did not initialize, do teardown on whatever did come up
            self._stop_infrastructure()
            raise
            
        # Initialize the actual runner. 
        # Requires config, pm, server and remote_runner
        self._init_runner()

        # Start the launch
        self.runner.launch()
        

    def shutdown_partial(self):
        node_list = [
            "twist_gate", "twist_filter", "pure_pursuit",
            "op_behavior_selector", "op_trajectory_evaluator",
            "op_motion_predictor", "op_trajectory_generator",
            "op_common_params", "op_global_planner",
            "wf_simulator", "world_to_map",
            "joint_state_publisher", "robot_state_publisher",
        ]
        for n in node_list:
            os.system("rosnode kill "+n)

p=[]

AUOWARE_DATA_PATH = os.getenv('AUOWARE_DATA_PATH', '/home/mtl/Haojie/MTL-Vehicle-Platform-data')
point_cloud_path = AUOWARE_DATA_PATH+'/maps/Mcity/LiDAR-map/MCity-lidar-map3_37.5_4.2_-35_yaw_0.027_pitch_-0.015.pcd'
vector_map_path = AUOWARE_DATA_PATH+'/maps/Mcity/HD-map/'
vector_map_files = [
    vector_map_path+'area.csv',
    vector_map_path+'dtlane.csv',
    vector_map_path+'intersection.csv',
    vector_map_path+'lane.csv',
    vector_map_path+'line.csv',
    vector_map_path+'node.csv',
    vector_map_path+'point.csv',
    vector_map_path+'pole.csv',
    vector_map_path+'poledata.csv',
    vector_map_path+'signaldata.csv',
    vector_map_path+'stopline.csv',
    vector_map_path+'vector.csv',
    vector_map_path+'wayarea.csv',
    vector_map_path+'whiteline.csv'
]
tf_path = AUOWARE_DATA_PATH+'/tf/default_world_map_tf.launch'

close_event = asyncio.Event() # the information waiting for triggering

shell_flag = False
async def start_sim(cmd_handler):
    # cmd_handler.run_process(['roslaunch', 'runtime_manager', 'runtime_manager.launch'], new_shell=shell_flag)
    cmd_handler.run_process(['roscore'], new_shell=shell_flag)
    time.sleep(5)
    # cmd_handler.run_process(['roslaunch', 'dbw_mkz_description', 'mkz.launch'])
    cmd_handler.run_process(['rosrun', 'map_file', 'points_map_loader', 'noupdate', point_cloud_path], new_shell=shell_flag)
    cmd_handler.run_process(['rosrun', 'map_file', 'vector_map_loader'] + vector_map_files, new_shell=shell_flag)
    # cmd_handler.run_process(['roslaunch', tf_path])
    cmd_handler.run_process(['rosrun', 'detected_objects_visualizer', 'visualize_detected_objects', '__name:=contour_track_visualization_01', '__ns:=/detection/contour_tracker' ], new_shell=shell_flag)
    print('load map successfully')
    time.sleep(5)
    # cmd_handler.run_process(['python', 'mcity_test_new.py'], new_shell=True)

    global p
    
    cli_args10 = [tf_path]

    cli_args0 = ['dbw_mkz_description', 'mkz.launch']

    cli_args1 = ['wf_simulator', 'wf_simulator.launch', 'initialize_source:=RVIZ', 'add_measurement_noise:=False', 'use_waypoints_for_z_position_source:=True', 'vehicle_model_type:=IDEAL_STEER', 'simulation_frame_id:=base_link', 'lidar_frame_id:=velodyne', 'sim_pose_name:=current_pose', 'sim_velocity_name:=current_velocity', 'vel_lim:=30', 'angvel_lim:=3', 'accel_rate:=1', 'angvel_rate:=1', 'vel_time_delay:=0.25', 'vel_time_constant:=0.6', 'angvel_time_delay:=0.2', 'angvel_time_constant:=0.4', 'steer_lim:=1', 'steer_rate_lim:=0.5', 'steer_time_delay:=0.24', 'steer_time_constant:=0.27', 'time_delay:=5.0']

    cli_args2 = ['op_global_planner', 'op_global_planner.launch', 'enableLaneChange:=False', 'enableReplan:=True', 'enableSmoothing:=True', 'enableRvizInput:=False', 'goalConfirmDistance:=30', 'pathDensity:=1', 'velocitySource:=1', f'route_file:={settings.VUT_autoware_route_path}', 'time_delay:=5.0']

    cli_args3 = ['op_local_planner', 'op_common_params.launch', 'horizonDistance:=120', 'maxLocalPlanDistance:=80', 'pathDensity:=0.5', 'rollOutDensity:=0.5', 'rollOutsNumber:=4', f'maxVelocity:={settings.VUT_autoware_max_speed}', 'maxAcceleration:=2', 'maxDeceleration:=-7', 'enableFollowing:=True', 'enableSwerving:=True', 'minFollowingDistance:=40', 'minDistanceToAvoid:=20', 'maxDistanceToAvoid:=8', 'enableStopSignBehavior:=False', 'enableTrafficLightBehavior:=True', 'enableLaneChange:=False', 'horizontalSafetyDistance:=1', 'verticalSafetyDistance:=1', 'velocitySource:=1', 'additionalBrakingDistance:=20', 'giveUpDistance:=-10.0', 'time_delay:=10.0']

    cli_args4 = ['op_local_planner', 'op_trajectory_generator.launch', 'samplingTipMargin:=4', 'samplingOutMargin:=12']

    cli_args5 = ['op_local_planner', 'op_motion_predictor.launch', 'enableCurbObstacles:=False', 'enableGenrateBranches:=False', 'max_distance_to_lane:=2', 'prediction_distance:=25', 'enableStepByStepSignal:=False', 'enableParticleFilterPrediction:=False', 'min_prediction_time:=10.0']

    cli_args6 = ['op_local_planner', 'op_trajectory_evaluator.launch', 'enablePrediction:=False']

    cli_args7 = ['op_local_planner', 'op_behavior_selector.launch']

    cli_args8 = ['pure_pursuit', 'pure_pursuit.launch', 'is_linear_interpolation:=True', 'publishes_for_steering_robot:=True', 'add_virtual_end_waypoints:=False', 'time_delay:=25.0']

    cli_args9 = ['twist_filter', 'twist_filter.launch', 'use_decision_maker:=False', 'time_delay:=25.0']

    roslaunch_file10 = roslaunch.rlutil.resolve_launch_arguments(cli_args10)[0]

    roslaunch_file0 = roslaunch.rlutil.resolve_launch_arguments(cli_args0)[0]

    roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0]
    roslaunch_args1 = cli_args1[2:]

    roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(cli_args2)[0]
    roslaunch_args2 = cli_args2[2:]

    roslaunch_file3 = roslaunch.rlutil.resolve_launch_arguments(cli_args3)[0]
    roslaunch_args3 = cli_args3[2:]

    roslaunch_file4 = roslaunch.rlutil.resolve_launch_arguments(cli_args4)[0]
    roslaunch_args4 = cli_args4[2:]

    roslaunch_file5 = roslaunch.rlutil.resolve_launch_arguments(cli_args5)[0]
    roslaunch_args5 = cli_args5[2:]

    roslaunch_file6 = roslaunch.rlutil.resolve_launch_arguments(cli_args6)[0]
    roslaunch_args6 = cli_args6[2:]

    roslaunch_file7 = roslaunch.rlutil.resolve_launch_arguments(cli_args7)[0]

    roslaunch_file8 = roslaunch.rlutil.resolve_launch_arguments(cli_args8)[0]
    roslaunch_args8 = cli_args8[2:]

    roslaunch_file9 = roslaunch.rlutil.resolve_launch_arguments(cli_args9)[0]
    roslaunch_args9 = cli_args9[2:]

    launch_files = [
        roslaunch_file10,
        roslaunch_file0,
        (roslaunch_file1, roslaunch_args1),
        (roslaunch_file2, roslaunch_args2),
        (roslaunch_file3, roslaunch_args3),
        (roslaunch_file4, roslaunch_args4),
        (roslaunch_file5, roslaunch_args5),
        (roslaunch_file6, roslaunch_args6),
        roslaunch_file7,
        (roslaunch_file8, roslaunch_args8),
        (roslaunch_file9, roslaunch_args9),
    ]

    print('starting')
    cmd_handler.ros_get_sumo_state()

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    parent = ROSLaunchParentNew(uuid, launch_files)

    for num in range(500):
        waitforsumo_index = 0
        while 1:
            if not cmd_handler.sumo_state:
                waitforsumo_index += 1
                time.sleep(0.5)
                print(f"Wait for SUMO start for {waitforsumo_index*0.5}s!")
            else:
                print(f"Wait for SUMO start for {waitforsumo_index*0.5}s!")
                break
        
        if num == 0:
            parent.start(auto_terminate=False)
        else:
            parent.restart()
        
        publish_initial_pose(
            x = settings.VUT_autoware_starting_point[0],
            y = settings.VUT_autoware_starting_point[1],
            w = settings.VUT_autoware_starting_point[2],
            z = settings.VUT_autoware_starting_point[3]
        )
        # publish_initial_pose(64.0746002197,77.3578414917,0.733722201637,0.67944957931)
        # publish_initial_pose(-1.50692558289,283.936279297,0.3589331575,-0.93336326714)
        # publish_initial_pose(-37.8328132629,159.603744507,0.686856946969,-0.726792635076)
        # publish_goal_pose(74.0641555786,309.015960693,0.709844733685,0.704358185911)
        # print('goal pose published')

        sleep_index = 0
        while 1:
            if not cmd_handler.sumo_state:
                print("SUMO is stopped!")
                break
            else:
                sleep_index += 1
                time.sleep(0.5)
            if sleep_index > 3000:
                break
        
        parent.shutdown_partial()
    
        print('current round:'+str(num+1)+'/500')

    print('Finish task')

    await close_event.wait() # waiting for the close event triggering to finally close the program
    print('closing')

async def main():
    cmd_handler = CMDHandler()
    task = asyncio.create_task(start_sim(cmd_handler))
    await task

if __name__ == '__main__':
    asyncio.get_event_loop().run_until_complete(main())
    