import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
import math

class numeric_dynamics(Node):

	def __init__(self):
		super().__init__('cannon_dynamics')

		self.publisher_ = self.create_publisher(Float64MultiArray, 'cannon/state', 10)
		self.sub_scheduler = self.create_subscription(Bool, '/scheduler', self.timer_callback, 1)

		self.cannon_default_data()
		self.init_calculations()

		#timer_period = 0.01  # seconds
		#self.timer = self.create_timer(timer_period, self.timer_callback)


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


	def analytic_step(self):
		self.vel[0] = self.vel_orig[0] + self.acc[0] * self.time
		self.vel[1] = self.vel_orig[1] + self.acc[1] * self.time

		self.pos[0] = self.pos_orig[0] + (self.vel_orig[0] + (0.5) * self.acc[0] * self.time) * self.time
		self.pos[1] = self.pos_orig[1] + (self.vel_orig[1] + (0.5) * self.acc[1] * self.time) * self.time
		
		if (self.pos[1] < 0.0):
			self.impactTime = (-self.vel_orig[1] - math.sqrt(self.vel_orig[1] * self.vel_orig[1] - 2 * self.pos_orig[1]))/self.acc[1]
			self.pos[0] = self.impactTime * self.vel_orig[0]
			self.pos[1] = 0.0
			self.vel[0] = 0.0
			self.vel[1] = 0.0

			if not self.impact:
				self.impact = True
				self.get_logger().info(f"\n\nIMPACT: t = {self.impactTime}, pos[0] = {self.pos[0]}\n\n")
        
		# Increment time by the time delta that matches the frequency 
		# specified in the scheduler node.
		self.time += 0.01
		

	def timer_callback(self,heartbeat):
		msg = Float64MultiArray()

		# single calculation step 
		self.analytic_step()

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