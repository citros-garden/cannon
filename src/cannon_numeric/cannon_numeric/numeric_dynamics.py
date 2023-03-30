import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
import math
import numpy as np

class numeric_dynamics(Node):

	def __init__(self):
		super().__init__('cannon_dynamics')

		self.cannon_default_data()
		self.init_calculations()

		self.publisher_ = self.create_publisher(Float64MultiArray, 'cannon/state', 10)
		self.sub_scheduler = self.create_subscription(Bool, '/scheduler', self.timer_callback, 1)

		# for a stand-alone version of this node (which is not dependant on a scheduler node),
		# comment out the previous line and uncomment the following two lines.
		# timer_period = 0.01  # seconds
		# self.timer = self.create_timer(timer_period, self.timer_callback)


	def cannon_default_data(self):
		self.pos_orig = [0.0, 0.0]
		self.pos = [0.0, 0.0]
		self.acc = [0.0, -9.81]
		self.time = 0.0
		self.impact = False
		self.impactTime = 0.0

		# declare parameters
		self.declare_parameter('init_speed', 50.0)
		self.declare_parameter('init_angle', 30.0)

		# get value from yaml
		self.init_speed = self.get_parameter('init_speed').get_parameter_value().double_value
		self.init_angle = self.get_parameter('init_angle').get_parameter_value().double_value


	def init_calculations(self):
		self.vel_orig = [math.cos(math.radians(self.init_angle))*self.init_speed, 
							math.sin(math.radians(self.init_angle))*self.init_speed]
		self.vel = [self.vel_orig[0], self.vel_orig[1]]


	def cannon_integ(self):
		# assume constant acceleration
		y0 = [self.acc[0], self.acc[0]]
		y1 = [self.acc[1], self.acc[1]]

		# integrate 
		new_vel0 = np.trapz(y0,dx=0.01) + self.vel[0]
		new_vel1 = np.trapz(y1,dx=0.01) + self.vel[1]
		new_pos0 = np.trapz([self.vel[0], new_vel0], dx=0.01) + self.pos[0]
		new_pos1 = np.trapz([self.vel[1], new_vel1], dx=0.01) + self.pos[1]

		# update state
		self.vel[0] = new_vel0
		self.vel[1] = new_vel1
		self.pos[0] = new_pos0
		self.pos[1] = new_pos1

		self.time += 0.01

		if (self.pos[1] < 0.0):
			self.impactTime = self.time
			if not self.impact:
				self.impact = True
				self.get_logger().info(f"\n\nIMPACT: t = {self.impactTime}, pos[0] = {self.pos[0]}\n\n")


	def timer_callback(self,heartbeat):
		msg = Float64MultiArray()

		# single calculation step 
		self.cannon_integ()

		msg.data = [self.pos[0], 
					self.pos[1], 
					self.vel[0], 
					self.vel[1]]

		# publish result
		self.publisher_.publish(msg)

		# for debugging: print published data in console
		if not self.impact:
			self.get_logger().info(f"Publishing: {msg.data}")


def main(args=None):
    rclpy.init(args=args)

    cannon_publisher = numeric_dynamics()

    rclpy.spin(cannon_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cannon_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
	main()