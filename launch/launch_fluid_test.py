from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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