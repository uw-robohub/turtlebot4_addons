#!/usr/bin/python3
import sys
import rclpy
from rclpy.node import Node
from copy import copy
import math
from geometry_msgs.msg import Twist, Accel, Wrench
from nav_msgs.msg import Odometry

class MinimalPublisher(Node):
    def __init__(self,case):
        super().__init__('minimal_publisher')
        self.get_logger().info("case %s" % case)
        self.case = case
        self.cmd_acc_publisher_ = self.create_publisher(Accel, '/cmd_acc', 1)
        self.cmd_wrench_publisher_ = self.create_publisher(Accel, '/cmd_wrench', 1)

        import rclpy.qos
        import rclpy.time
        qos_profile_sensor_data = copy(rclpy.qos.qos_profile_sensor_data)
        qos_policy = copy(rclpy.qos.qos_profile_sensor_data)

        self.odom_subscription = self.create_subscription(
                Odometry,
                '/odom',
                self.odom_listener_callback,
                qos_profile_sensor_data)
        self.odom_subscription  # prevent unused variable warning

        self.timer_period = 0.05
        self.state = "off"
        self.timer = self.create_timer(self.timer_period, self.timer_callback)


    def timer_callback(self):
        if self.case == "test_1":
            msg = Accel()
            if not hasattr(self,'init_time'):
                self.init_time = self.get_clock().now()
            f = 0.5
            msg.linear.x = math.sin(2*math.pi*f*(self.get_clock().now() - self.init_time).nanoseconds/1e9)
            self.cmd_acc_publisher_.publish(msg)
        elif self.case == "test_2":
            if not hasattr(self,'odom'): return
            #self.get_logger().info('Odom %s' % msg)
            if not hasattr(self, 'test_2_initial_state'):
                self.test_2_initial_state = copy(self.odom)
            error = self.test_2_initial_state.pose.pose.position.x + 1. - self.odom.pose.pose.position.x
            self.get_logger().info('state %s' % self.odom.pose.pose.position.x)
            self.get_logger().info('error %s' % error)
            derror = (self.test_2_initial_state.twist.twist.linear.x - self.odom.twist.twist.linear.x)
            msg_out = Accel()
            p_gain = 10.
            inertia = 1.
            d_gain = 0.7 * 2. * math.sqrt(p_gain * inertia)
            msg_out.linear.x = p_gain * error + d_gain * derror
            self.get_logger().info('comma %s' % msg_out.linear.x)
            self.cmd_acc_publisher_.publish(msg_out)

        else:
            raise Exception("Not implemented")

    def odom_listener_callback(self, msg):
        #self.get_logger().info("odom %s" % msg.pose.pose.position.x)
        self.odom = msg




def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher(sys.argv[1])

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





