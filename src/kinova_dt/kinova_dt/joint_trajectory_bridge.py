#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class JointTrajectoryBridge(Node):
    def __init__(self):
        super().__init__('joint_trajectory_bridge')
        # parameters (override from launch)
        self.input_topic = self.declare_parameter('input_topic', '/kinova_gui/joint_states').value
        self.output_topic = self.declare_parameter('output_topic', '/joint_trajectory_controller/joint_trajectory').value
        self.time_from_start = float(self.declare_parameter('time_from_start', 1.0).value)

        self.sub = self.create_subscription(JointState, self.input_topic, self.joint_cb, 10)
        self.pub = self.create_publisher(JointTrajectory, self.output_topic, 10)
        self.get_logger().info(f'Bridge: {self.input_topic} -> {self.output_topic} (t={self.time_from_start}s)')

    def joint_cb(self, msg: JointState):
        if not msg.name or not msg.position:
            return
        traj = JointTrajectory()
        traj.joint_names = list(msg.name)
        pt = JointTrajectoryPoint()
        # copy positions (make sure lengths match)
        positions = list(msg.position)
        pt.positions = positions
        # small safety: set time_from_start
        t = Duration()
        t.sec = int(self.time_from_start)
        t.nanosec = int((self.time_from_start - t.sec) * 1e9)
        pt.time_from_start = t
        traj.points = [pt]
        self.pub.publish(traj)
        self.get_logger().debug(f'Published trajectory for {len(positions)} joints')

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
