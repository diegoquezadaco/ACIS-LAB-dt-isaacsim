#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

class JointControlGUI(Node):
    def __init__(self):
        super().__init__('joint_control_gui')

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
            'joint_7'
        ]

        # Predefined poses dictionary:
        # Each pose is a list of 7 joint angles (radians), matching joint_names order
        self.poses = {
            1: ("Home Position",       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            2: ("Ready Position",      [0.0, -1.57, 0.0, -1.57, 0.0, 1.57, 0.0]),
            3: ("Extended Forward",    [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            4: ("Pick Pose",           [0.5, -0.5, 0.5, -1.0, 0.5, 0.5, 0.0]),
            5: ("Place Pose",          [-0.5, 0.5, -0.5, 1.0, -0.5, -0.5, 0.0]),
        }

        self.window_name = "Select Pose: Press 1-5 to move robot, ESC to quit"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        # Show the menu once at start
        self.show_menu()

        self.should_shutdown = False

    def show_menu(self):
        # Create a black image to draw the menu
        menu_img = 255 * np.ones((300, 400, 3), dtype=np.uint8)

        y = 30
        for key in sorted(self.poses.keys()):
            text = f"{key}: {self.poses[key][0]}"
            cv2.putText(menu_img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)
            y += 40

        cv2.imshow(self.window_name, menu_img)
        cv2.waitKey(1)

    def send_pose(self, pose_key):
        if pose_key not in self.poses:
            self.get_logger().warn(f"Pose {pose_key} not found")
            return

        pose_name, joint_positions = self.poses[pose_key]
        duration_sec = 30  # Fixed 30 seconds per your request

        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = duration_sec
        point.time_from_start.nanosec = 0

        traj_msg.points.append(point)

        self.publisher.publish(traj_msg)
        self.get_logger().info(f"Sent pose '{pose_name}' with duration {duration_sec}s")

    def run(self):
        while rclpy.ok() and not self.should_shutdown:
            rclpy.spin_once(self, timeout_sec=0.1)
            key = cv2.waitKey(50) & 0xFF

            if key == 27:  # ESC key to quit
                self.get_logger().info("ESC pressed, shutting down...")
                self.should_shutdown = True
            elif key in [ord(str(i)) for i in self.poses.keys()]:
                pose_num = int(chr(key))
                self.send_pose(pose_num)
            else:
                # Ignore other keys
                pass

def main(args=None):
    import numpy as np
    rclpy.init(args=args)
    node = JointControlGUI()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
