import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool


class scheduler(Node):

	def __init__(self):
		super().__init__('scheduler')

		self.sub_debug = self.create_subscription(Bool, '/debug', self.debug_step, 1)

		self.publisher_ = self.create_publisher(Bool, 'scheduler', 10)
		timer_period = 0.01  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.debug_mode = False

	def timer_callback(self):
		msg = Bool()
		
		# heart beat
		msg.data = True
		if not self.debug_mode:
			self.publisher_.publish(msg)

	# callback to handle messages from foxglove control panel
	def debug_step(self, msg):
		if msg.data:
			self.debug_mode = True
			self.publisher_.publish(msg)
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