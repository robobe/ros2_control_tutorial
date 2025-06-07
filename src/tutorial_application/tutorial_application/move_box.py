#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from geometry_msgs.msg import Pose

#!/usr/bin/env python3

class MoveBoxNode(Node):
    def __init__(self):
        super().__init__('move_box_node')
        self.cli = self.create_client(SetEntityPose, '/world/my_world/set_pose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /world/my_world/set_pose service...')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.z = 0.5
        self.direction = 1  # 1 for up, -1 for down

    def timer_callback(self):
        req = SetEntityPose.Request()
        req.entity.name = 'green_simple_box'
        req.pose.position.y = 0.0
        req.pose.position.x = 8.0
        req.pose.position.z = self.z

        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)

        # Update z for next call
        self.z += 0.1 * self.direction
        if self.z >= 4.0:
            self.z = 4.0
            self.direction = -1
        elif self.z <= 0.5:
            self.z = 0.5
            self.direction = 1

    def response_callback(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().warn(f"Service call failed: {response.status_message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MoveBoxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()