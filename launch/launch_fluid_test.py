from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
	return LaunchDescription([
		DeclareLaunchArgument('my_package_path', default_value='$(pwd)',description='Path to my ROS2 package'),
		SetEnvironmentVariable('PYTHONPATH', value=[LaunchConfiguration('my_package_path') + '/src/anrs', '$PYTHONPATH']),

		Node(
			package='anrs',
			namespace='anrs_fluid',
			executable='stepper_driver_node',
			name='fluid_stepper_node'
			parameters=[{'pinStep':23,'pinDir':24,'pinEn':25,
				     'pulsesPerStep':1,'stepsPerRev':200,
				     'accel0':1000,'accelMax':5000,'accelMin':1,
				     'speed0':2000,'speedMax':5000,'speedMin'200}]
		)
	])
