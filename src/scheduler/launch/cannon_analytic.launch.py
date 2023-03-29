import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	ld = LaunchDescription()
	config = os.path.join(
	get_package_share_directory('cannon_analytic'),
	'config',
	'params.yaml'
	)

	sched_node=Node(
	package = 'scheduler',
	name = 'scheduler',
	executable = 'scheduler',
	parameters = [config]
	)

	cannon_node=Node(
	package = 'cannon_analytic',
	name = 'cannon_analytic',
	executable = 'cannon_dynamics',
	parameters = [config]
	)

	ld.add_action(sched_node)
	ld.add_action(cannon_node)
	return ld