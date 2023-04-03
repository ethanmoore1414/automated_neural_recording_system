from argparse import Action
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from anrs.Stepper_Driver_TMC2208 import Stepper_Driver

#######################################################
# Constants
#######################################################
#Directions of rotation from perspective with motor shaft pointed towards you with right hand rule
POSITIVE = True         #counterclockwise
NEGATIVE = False          #clockwise
#Modes of movement
ABSOLUTE = False         #rotate relative to the 0 position
RELATIVE = True          #rotate relative to the current position

#####################################################################################
# Wrapped Stepper Main Class
#####################################################################################
class Stepper(Node):
	def __init__(self, name, pinStep, pinDir, pinEn, speedMax, pulsesPerStep, stepsPerRev):
		# create the node and get the parameters likely set in the launch file
		super().__init__('stepper_node')
		params = self.get_parameter('stepper_params')

		# set up the stepper driver object
		self.driver = Stepper_Driver(params['pinStep'],params['pinDir'],params['pinEn'],
									 params['pulsesPerStep'],params['stepsPerRev'],
									 params['accel0'],params['accelMax'],params['accelMin']
									 params['speed0'],params['speedMax'],params['speedMin'])
		self.driver.setDirection(POSITIVE)
		self.driver.setMovementMode(RELATIVE)
		self.driver.setEnabled(True)

		# create the node's subscribers
		self.subPoseTarget = self.create_subscription(Float32, "%sPoseTargets"%(self.name), self.runToRevCount)
		self.subSpeedTarget = self.create_subscription(Float32, "%sSpeedTargets"%(self.name), self.setRevSpeedTarget)
		self.subLimit = self.create_subscription(Bool, "%sLimit"%(self.name), self.setStopped)
		# create the node's publishers
		self.pubPose = self.create_publisher(Float32, "%sPose"%(self.name), 10)
		self.pubStatus = self.create_publisher(Bool, "%sStatus"%(self.name), 10)

		# log initilizaiton completion
		rclpy.get_logger().info("stepper %s is initialized and ready"%(stepper_params['Name']))		

	def runToRevCount(self, value):
		self.driver.runToRevCount(value.data)

	def setRevSpeedTarget(self, value):
		self.driver.setRevSpeedTarget(value.data)

def main(args=None):
	rclpy.get_logger().info('setting up stepper')
	rclpy.init(args=args)
	stepper_node = Stepper()
	rclpy.spin(stepper_node)

if __name__ == '__main__':
	main()
