import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, Bool


class cannon_dynamics(Node):

    def __init__(self):
        super().__init__('cannon_dynamics')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'cannon/state', 10)

        self.sub_scheduler = self.create_subscription(Bool, '/scheduler', self.timer_callback, 1)

        # declare parameters
        self.declare_parameter('initial_velocity', 50.0)
        self.declare_parameter('angle', 30.0)

        # get value from yaml
        self.initial_velocity = self.get_parameter('initial_velocity').get_parameter_value().double_value
        self.angle = self.get_parameter('angle').get_parameter_value().double_value

        #timer_period = 0.01  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self,heartbeat):
        msg = Float64MultiArray()

        # TODO: calculate and publish data here
        msg.data = [42.69]
        self.publisher_.publish(msg)

        # access published data from commandline
        self.get_logger().info(f"Publishing: {msg.data}") 

        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    cannon_publisher = cannon_dynamics()

    rclpy.spin(cannon_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cannon_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
	main()