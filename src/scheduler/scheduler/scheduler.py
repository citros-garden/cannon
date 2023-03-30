import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool


class scheduler(Node):

	def __init__(self):
		super().__init__('scheduler')

		# publish and subscribe
		self.sub_debug = self.create_subscription(Bool, '/debug', self.debug_step, 1)
		self.publisher_ = self.create_publisher(Bool, 'scheduler', 10)

		# declare parameter
		self.declare_parameter('dt', 0.01) # seconds

		# create and start timer
		timer_period = self.get_parameter('dt').get_parameter_value().double_value
		self.timer = self.create_timer(timer_period, self.timer_callback)
		
		self.debug_mode = False


	def timer_callback(self):
		# if not in debug mode, send heartbeat every timeout
		if not self.debug_mode:
			msg = Bool()
			msg.data = True
			self.publisher_.publish(msg)


	# callback to handle messages from foxglove control panel
	def debug_step(self, msg):
		if msg.data:
			if self.debug_mode:
				# single step
				self.publisher_.publish(msg)
			else:
				# pause
				self.debug_mode = True
			
		else:
			self.debug_mode = False


def main(args=None):
    rclpy.init(args=args)

    sched = scheduler()

    rclpy.spin(sched)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sched.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
	main()