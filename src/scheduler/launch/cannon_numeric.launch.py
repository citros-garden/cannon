from launch import LaunchDescription, launch_description_sources
from launch_ros.actions import Node
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
import os
from launch_ros.actions import Node

def generate_launch_description():
	ld = LaunchDescription()

	config_sched = os.path.join(
	get_package_share_directory('scheduler'),
	'config',
	'params.yaml'
	)

	config_numeric = os.path.join(
	get_package_share_directory('cannon_analytic'),
	'config',
	'params.yaml'
	)

	sched_node=Node(
	package = 'scheduler',
	name = 'scheduler',
	executable = 'scheduler',
	parameters = [config_sched]
	)

	cannon_numeric_node=Node(
	package = 'cannon_numeric',
	name = 'numeric_dynamics',
	executable = 'numeric_dynamics',
	parameters = [config_numeric]
	)

	bridge_dir = get_package_share_directory('rosbridge_server')
	bridge_launch =  IncludeLaunchDescription(launch_description_sources.FrontendLaunchDescriptionSource(bridge_dir + '/launch/rosbridge_websocket_launch.xml')) 

	sys_shut_down = RegisterEventHandler(OnProcessExit(
		target_action=cannon_numeric_node,
        on_exit=[
                    LogInfo(msg=(f'The Scenario has ended!')),
                    EmitEvent(event=Shutdown(
                        reason='Finished'))
		        ]		
	    ))

	ld.add_action(sched_node)
	ld.add_action(cannon_numeric_node)
	ld.add_action(bridge_launch)
	ld.add_action(sys_shut_down)
	
	return ld