#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_system_default
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

TOPIC_SET_POINT = "set_point"
PARAM_RATE = "rate"
PARAM_SETPOINT = "setpoint"

class MyNode(Node):
    def __init__(self):
        node_name="setpoint_alterer"
        super().__init__(node_name)
        self.declare_parameter(PARAM_SETPOINT, 1.0)
        self.declare_parameter(PARAM_RATE, 5.0)
        self.state = 1
        self.pub_set_point = self.create_publisher(Float32, TOPIC_SET_POINT, qos_profile=qos_profile_system_default)
        self.timer = self.create_timer(self.get_parameter(PARAM_RATE).value, self.timer_callback)
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().info("Hello alter_setpoint node")

    def parameter_callback(self, params):
        param: Parameter
        for param in params:
            if param.name == PARAM_RATE:
                self.timer.cancel()
                self.timer = self.create_timer(param.value, self.timer_callback)
                self.get_logger().info(f'update rate to {param.value}')

        return SetParametersResult(successful=True)

    def timer_callback(self):
        self.state *= -1
        current_setpoint = self.get_parameter(PARAM_SETPOINT).value
        new_setpoint = current_setpoint * self.state
        msg = Float32(data=new_setpoint)
        self.pub_set_point.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()