import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool, Float32
from anrs.COPYstepper_driver_TMC2208 import StepperDriver

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
		super().__init__('stepper')
		self.stepper = StepperDriver(pinStep, pinDir, pinEn, speedMax, pulsesPerStep, stepsPerRev)
		self.name = name
		self.subPose = self.create_subscription(Float32, "%sPoseTargets"%(self.name), self.runToRevCount)
		self.subSpeed = self.create_subscription(Float32, "%sSpeedTargets"%(self.name), self.setPulseSpeedTarget)
		self.subPowered = self.create_subscription(Bool, "%sPowered"%(self.name), self.setPowered)
		self.subLimit = self.create_subscription(Bool, "%sLimit"%(self.name), self.setStopped)
		self.pubRev = self.create_publisher(Float32, "%sRevCount"%(self.name), 10)
		self.pubStatus = self.create_publisher(Bool, "%sStatus"%(self.name), 10)
		self.subscription

	#-----------------------------------------------------------------------
	# destructor
	#-----------------------------------------------------------------------
	def _del_(self):
		del self.stepper

	#-----------------------------------------------------------------------
	# returns the motor's pin for step signal
	#-----------------------------------------------------------------------
	def getPinStep(self):
		return self.stepper.getPinStep()

	#-----------------------------------------------------------------------
	# sets the GPIO pin for motor step signal
	#-----------------------------------------------------------------------
	def setPinStep(self, pinStep):
		self.stepper.setPinStep(pinStep)

	#-----------------------------------------------------------------------
	# returns GPIO pin for motor step signal
	#-----------------------------------------------------------------------
	def getPinDir(self):
		return self.stepper.getPinDir()

	#-----------------------------------------------------------------------
	# sets GPIO pin for direction signal
	#-----------------------------------------------------------------------
	def setPinDir(self, pinDir):
		self.stepper.setPinEn(pinDir)

	#-----------------------------------------------------------------------
	# returns GPIO pin for enable signal
	#-----------------------------------------------------------------------
	def getPinEn(self):
		return self.stepper.getPinEn()

	#-----------------------------------------------------------------------
	# sets GPIO pin for enable signal
	#-----------------------------------------------------------------------
	def setPinEn(self, pinEn):
		self.stepper.setPinEn(pinEn)

	#-----------------------------------------------------------------------
	# returns set direction of rotation
	#-----------------------------------------------------------------------
	def getDirection(self):
		return self.stepper.getDirection()

	#-----------------------------------------------------------------------
	# sets direction of rotation
	#-----------------------------------------------------------------------
	def setDirection(self, direction):
		self.stepper.setDirection(direction)

	#-----------------------------------------------------------------------
	# returns enablement of motor
	#-----------------------------------------------------------------------
	def getEnabled(self):
		return self.stepper.getEnabled()

	#-----------------------------------------------------------------------
	# sets enablement of motor
	#-----------------------------------------------------------------------
	def setEnabled(self, en):
		self.stepper.setEnabled(en)

	#-----------------------------------------------------------------------
	# returned powered state of the motor
	#-----------------------------------------------------------------------
	def getPowered(self):
		return self.stepper.getPowered()

	#-----------------------------------------------------------------------
	# sets powered state of the motor
	#-----------------------------------------------------------------------
	def setPowered(self, powered):
		self.stepper.setPowered(powered.data)

	#-----------------------------------------------------------------------
	# returns set movement mode
	#-----------------------------------------------------------------------
	def getMovementMode(self):
		return self.stepper.getMovementMode()

	#-----------------------------------------------------------------------
	# sets movement mode ABSOLUTE: False; Relative: True
	#-----------------------------------------------------------------------
	def setMovementMode(self, movementMode):
		self.stepper.setMovementMode(movementMode)

	#-----------------------------------------------------------------------
	# returns set movement mode
	#-----------------------------------------------------------------------
	def getActive(self):
		return self.stepper.getActive()

	#-----------------------------------------------------------------------
	# returns set movement mode
	#-----------------------------------------------------------------------
	def getSeeking(self):
		return self.stepper.getSeeking()

	#-----------------------------------------------------------------------
	# returns stopped status of the motor
	#-----------------------------------------------------------------------
	def getStopped(self):
		self.stepper.getStopped()

	#-----------------------------------------------------------------------
	# sets the stopped status of the motor
	#-----------------------------------------------------------------------
	def setStopped(self, stopped):
		self.stepper.setStopped(stopped.data)

	#-----------------------------------------------------------------------
	# returns motor pulses per step (microstepping)
	#-----------------------------------------------------------------------
	def getPulsesPerStep(self):
		return self.stepper.getPulsesPerStep()

	#-----------------------------------------------------------------------
	# sets motor pulses per step (microstepping)
	#-----------------------------------------------------------------------
	def setPulsesPerStep(self, pulses):
		self.stepper.setPulsesPerStep(pulses)

	#-----------------------------------------------------------------------
	# returns motors steps per revolution
	#-----------------------------------------------------------------------
	def getStepsPerRev(self):
		return self.stepper.getStepsPerRevolution()

	#-----------------------------------------------------------------------
	# sets motors steps per revolution
	#-----------------------------------------------------------------------
	def setStepsPerRev(self, steps):
		self.stepper.setStepsPerRevolution(steps)

	#-----------------------------------------------------------------------
	# returns motors absolute count of occurred pulses
	#-----------------------------------------------------------------------
	def getPulseCountAbs(self):
		return self.stepper.getPulseCountAbs()

	#-----------------------------------------------------------------------
	# sets motors absolute count of occurred pulses
	#-----------------------------------------------------------------------
	def setPulseCountAbs(self, count):
		self.stepper.setPulseCountAbs(count)

	#-----------------------------------------------------------------------
	# returns motors absolute count of occurred pulses
	#-----------------------------------------------------------------------
	def getRevCountAbs(self):
		return self.stepper.getRevCountAbs()

	#-----------------------------------------------------------------------
	# sets motors absolute count of occurred rotations
	#-----------------------------------------------------------------------
	def setRevCountAbs(self, count):
		self.stepper.setRevCountAbs(count)

	#-----------------------------------------------------------------------
	# returns the target motor speed in pulses per second
	#-----------------------------------------------------------------------
	def getPulseSpeedTarget(self):
		return self.stepper.getPulseSpeedTarget()

	#-----------------------------------------------------------------------
	# sets the target motor speed in pulses per second
	#-----------------------------------------------------------------------
	def setPulseSpeedTarget(self, speed):
		self.stepper.setPulseSpeedTarget(speed)

	#-----------------------------------------------------------------------
	# returns the pulse acceleration in pulses per second per second
	#-----------------------------------------------------------------------
	def getPulseAcceleration(self):
		return self.stepper.getPulseAcceleration()

	#-----------------------------------------------------------------------
	# sets the target motor speed in pulses per second
	#-----------------------------------------------------------------------
	def setPulseAcceleration(self, acceleration):
		self.stepper.setPulseAcceleration(acceleration)

	#-----------------------------------------------------------------------
	# runs the motor to the given position.
	# with acceleration and deceleration
	# blocks the code until finished or stopped from a different thread!
	# returns true when the movement if finshed normally and false,
	# when the movement was stopped
	#-----------------------------------------------------------------------
	def runToRevCount(self, count):
		self.pubStatus.publish(True)
		rospy.loginfo("requesting %s to rotate %s revolutions"%(self.name, count.data))
		self.stepper.runToRevCount(count.data)
		self.pubStatus.publish(False)

def main(args=None):
	print('setting up steppers')
	rclpy.init(args=args)

	stepper = Stepper()

	args = rclpy.myargv(argv=sys.argv)
	if len(args) != 2:
		rclpy.loginfo("incorrect argument assignment")
		sys.exit(1)
	for steppers in rclpy.get_param("steppers"):
		if (steppers['Name'] == args[1]):
			stepper = steppers
	driver = stepper_driver_TMC2208_ROSwrapper(stepper['Name'],stepper['pinStep'],stepper['pinDir'],stepper['pinEn'],stepper['speedMax'],stepper['pulsesPerStep'],stepper['stepsPerRev'])
	driver.setPulseAcceleration(stepper['accel0'])
	driver.setPulseSpeedTarget(stepper['speedMax'])
	driver.setMovementMode(RELATIVE)
	driver.setEnabled(True)
	driver.setPowered(Bool(True))
	driver.setDirection(POSITIVE)
	rclpy.loginfo("%s is initialized and ready"%(stepper['Name']))
	rclpy.loginfo("Stepper initialization complete")
	rate = rclpy.Rate(10)
	while not rclpy.is_shutdown():
		driver.pubRev.publish(driver.getRevCountAbs())
		rate.sleep()
	# optionally use rclpy.spin(stepper) instead

if __name__ == '__main__':
	main()
