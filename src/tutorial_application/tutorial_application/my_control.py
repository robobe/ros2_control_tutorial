#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, Float32, Float32MultiArray
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from tf_transformations import euler_from_quaternion
from rclpy.parameter import Parameter

TOPIC_IMU = "/imu_broadcaster/imu"
TOPIC_VELOCITY_CMD = "/effort_controller/commands"
TOPIC_SET_POINT = "set_point"
TOPIC_PID_OUTPUT = "pid_output"

class PIDController:
    def __init__(self, p: float, i: float, d: float):
        self.kp = p
        self.ki = i
        self.kd = d
        self.integral = 0.0
        self.prev_error = 0.0
        self.min_limit = -200
        self.max_limit = 200
        self.error = 0
        self.out_p = 0.0
        self.out_i = 0.0
        self.out_d = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def get_error(self):
        return self.error
    
    def compute(self, setpoint: float, measurement: float, dt: float):
        self.error = setpoint - measurement
        self.integral += self.error * dt * self.ki
        derivative = (self.error - self.prev_error) / dt if dt > 0 else 0.0

        self.out_p = self.kp * self.error
        self.out_i = self.integral
        self.out_d = self.kd * derivative

        output = self.kp * self.error + self.ki * self.integral + self.kd * derivative
        self.prev_error = self.error
        output = max(min(self.max_limit, output), self.min_limit)
        
        return output
    

class Controller(Node):
    def __init__(self):
        node_name="controller"
        super().__init__(node_name)
        self.controller = PIDController(4.0, 0.02, 0)
        
        self.sub_set_point = self.create_subscription(Float32, TOPIC_SET_POINT, self.__set_point_handler, qos_profile=qos_profile_system_default)
        self.sub_imu = self.create_subscription(Imu, TOPIC_IMU, self.__imu_handler, qos_profile=qos_profile_system_default)
        self.pub_effort = self.create_publisher(Float64MultiArray, TOPIC_VELOCITY_CMD, qos_profile=qos_profile_system_default)
        self.pub_pid_output = self.create_publisher(Float32MultiArray, TOPIC_PID_OUTPUT, qos_profile=qos_profile_system_default)
        self.declare_parameter("pid_p", value=4)
        self.declare_parameter("pid_i", value=0.02)
        self.declare_parameter("pid_d", value=0.0)
        # self.create_timer(2.0, self.__publish_velocity)
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.__set_point = None
        self.__last_time = self.get_clock().now()
        self.get_logger().info("Hello ROS2")


    def parameter_callback(self, params):
        param: Parameter
        for param in params:
            if param.name == 'pid_p':
                self.controller.kp = param.value
                self.get_logger().info(f'update p  {param.value}')

            if param.name == 'pid_i':
                self.controller.ki = param.value
                self.get_logger().info(f'update i  {param.value}')

            if param.name == 'pid_d':
                self.controller.kd = param.value
                self.get_logger().info(f'update d  {param.value}')
        return SetParametersResult(successful=True)
    
    def __set_point_handler(self, msg: Float32):
        self.__set_point = msg.data
        # self.get_logger().info(f"---- {self.__set_point} ---")

    def __publish_effort(self, vel):
            msg = Float64MultiArray()
            msg.data = [vel, 0.0]  # Example velocity values
            self.pub_effort.publish(msg)
            # self.get_logger().info("Published velocity command")

    def __imu_handler(self, msg: Imu):
        r, p, y =  euler_from_quaternion([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
            
        ])
        # self.get_logger().info(f"{r=} , {p=}, {y}")
        
        dt = (self.get_clock().now() - self.__last_time).nanoseconds * 1e-6

        self.__last_time = self.get_clock().now()
        if self.__set_point is not None:
            effort = self.controller.compute(self.__set_point, p, dt)
            self.get_logger().info(f"pid output: {effort}")
            self.__publish_effort(effort)
            xxx = Float32MultiArray()
            # sp, error, output, p, i , d , 
            xxx.data = [self.__set_point, 
                        self.controller.error, 
                        effort,
                        self.controller.out_p,
                        self.controller.out_i,
                        self.controller.out_d
                        ]
            self.pub_pid_output.publish(xxx)


        

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()