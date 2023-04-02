from launch import LaunchDescription, launch_description_sources
from launch_ros.actions import Node
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

	ld.add_action(sched_node)
	ld.add_action(cannon_numeric_node)
	ld.add_action(bridge_launch)

	return ld