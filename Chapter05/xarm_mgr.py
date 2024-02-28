import rclpy
import xarm
import time
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray, Int32

HighCarry=[9999,500,195,858,618,9999]
MidCarry=[9999, 500, 500, 807, 443, 9999]
Grasp = [100,500,151,553,117,9999]
GraspClose=[700,9999,9999,9999,9999,9999]
Align=[500,500,500,500,500,500]
Convert = [[-90,0],[-45,250],[0,500],[45,750],[]]


class xarmControl(Node):
	def __init__(self):
		super().__init__('xarm_manager')   # node name
		self.publisher = self.create_publisher(Int32MultiArray, 'xarm_pos', 10)
		self.armAngPub = self.create_publisher(Int32MultiArray, 'xarm_angle', 10)
		self.cmdSubscribe = self.create_subscription(String, 'RobotCmd', self.cmdCallback,10)
		self.wristSubscribe = self.create_subscription(Int32, 'xarmWrist', self.wristCallback,10)
		self.effSubscribe = self.create_subscription(Int32, 'xarmEffector', self.effCallback,10)
		self.baseSubscribe = self.create_subscription(Int32, 'xarmBase', self.baseCallback,10)
		self.baseSubscribe = self.create_subscription(Int32MultiArray, 'newArmPos', self.moveArmCallback,10)
		#self.posSubscribe = self.create_subscription(Int32MultiArray, 'xarmPos', self.effCallback,10)
		timer_period = 1.0  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0  # counter
		try:
			self.arm = xarm.Controller('USB')
			print("ARM OPEN")
		except:
			self.get_logger().error("xarm_manager init NO ARM DETECTED")
			self.arm = None
			print("ERROR init: NO ARM DETECTED")
		return
		
	def timer_callback(self):
		msg = Int32MultiArray()
		# call arm and get positions
		armPos=[]
		for i in range(1,7):
			armPos.append(self.arm.getPosition(i))
		msg.data = armPos
		self.publisher.publish(msg)
		# get arm positions in degrees
		armPos=[]
		for i in range(1,7):
			armPos.append(int(self.arm.getPosition(i, True)))
		msg.data = armPos
		#print(armPos)
		self.armAngPub.publish(msg)
		
	def cmdCallback(self, msg):
		self.get_logger().info("xarm rec cmd %s" % msg.data)
		robotCmd = msg.data
		if robotCmd=="ARM HIGH_CARRY":
			self.setArm(HighCarry)
		if robotCmd=="ARM MID_CARRY":
			self.setArm(MidCarry)
		if robotCmd=="ARM GRASP_POS":
			self.setArm(Grasp)
		if robotCmd=="ARM GRASP_CLOSE":
			self.setArm(GraspClose)
		if robotCmd=="ARM ALIGN":
			self.setArm(Align)
			
	def wristCallback(self, msg):
		# changed to use angles not servo units
		try:
			newArmPos = int(msg.data)
		except ValueError:
			self.get_logger().info("Invalid xarm wrist cmd %s" % msg.data)
			print("invalid wrist cmd ", msg.data)
			return
		# set limits
		newArmPos = float(min(90.0,newArmPos))
		newArmPos = float(max(-90.0,newArmPos))
		self.arm.setPosition(2,newArmPos, True)
	
	def effCallback(self, msg):
	# set just the end effector position
		try:
			newArmPos = int(msg.data)
		except ValueError:
			self.get_logger().info("Invalid xarm effector cmd %s" % msg.data)
			return
		# set limits
		newArmPos = min(1000,newArmPos)
		newArmPos = max(0,newArmPos)
		self.arm.setPosition(1,newArmPos)
	
	def baseCallback(self, msg):
	# set just the base azimuth position
		try:
			newArmPos = int(msg.data)
		except ValueError:
			self.get_logger().info("Invalid xarm base cmd %s" % msg.data)
			return
		# set limits
		newArmPos = min(1000,newArmPos)
		newArmPos = max(0,newArmPos)
		self.arm.setPosition(6,newArmPos)

	def moveArmCallback(self,msg):
		self.setArm(msg.data)
			
	def setArm(self,armPos):
		for i in range(5,0,-1):
			if armPos[i]<9999:   # we use 9999 to show we don't want to move that joint
				self.arm.setPosition(i+1, armPos[i],1300)
			#print("set ",i+1,"to ",armPos[i])
			time.sleep(0.1)
		if armPos[0]<9999:
			self.arm.setPosition(1,armPos[0],1300)
		#print("set  1 to ",armPos[0])			
##   END OF xarmControl Class Definition  ##

## MAIN
#######################MAIN####################################
rclpy.init()
print("Arm Control Active")
xarmCtr = xarmControl()


# spin ROS 2
rclpy.spin(xarmCtr)

# destroy node explicitly
xarmCtr.destroy_node()
rclpy.shutdown()

