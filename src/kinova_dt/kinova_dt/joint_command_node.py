from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rclpy
from rclpy.node import Node

class JointCommandRepublisher(Node):
    def __init__(self):
        super().__init__('joint_command_republisher')
        self.sub = self.create_subscription(JointTrajectory, '/isaac_joint_command', self.cmd_cb, 10)
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)

    def cmd_cb(self, msg):
        # Forward the command to real robot
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = JointCommandRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

