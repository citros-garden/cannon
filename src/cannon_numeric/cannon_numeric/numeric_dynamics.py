import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from cannon_numeric.cannon import cannon

class numeric_dynamics(Node):

	def __init__(self):
		super().__init__('cannon_dynamics')

		# declare and init cannon model
		self.cann = cannon(self.get_logger())

		# declare parameters
		self.declare_parameter('init_speed', 50.0)
		self.declare_parameter('init_angle', 30.0)
		self.declare_parameter('dt', 0.01)

		# get parameter values from yaml
		self.cann.init_speed = self.get_parameter('init_speed').get_parameter_value().double_value
		self.cann.init_angle = self.get_parameter('init_angle').get_parameter_value().double_value
		self.cann.dt = self.get_parameter('dt').get_parameter_value().double_value

		# publish and subscribe
		self.publisher_ = self.create_publisher(Float64MultiArray, 'cannon/state', 10)
		self.sub_scheduler = self.create_subscription(Bool, '/scheduler', self.timer_callback, 1)

		# for a stand-alone version of this node (which is not dependant on a scheduler node):
		# - comment out the previous line 
		# - uncomment the following two lines
		# - remove the heartbeat parameter from timer_callback
		# timer_period = self.cann.dt  # seconds
		# self.timer = self.create_timer(timer_period, self.timer_callback)


	def timer_callback(self,heartbeat):
		msg = Float64MultiArray()

		# single calculation step 
		if not self.cann.impact:
			self.cann.cannon_integ()

		msg.data = [self.cann.pos[0], 
			    self.cann.pos[1], 
			    self.cann.vel[0], 
			    self.cann.vel[1]]

		# publish result
		self.publisher_.publish(msg)

		# for debugging: print published data in console
		if not self.cann.impact:
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