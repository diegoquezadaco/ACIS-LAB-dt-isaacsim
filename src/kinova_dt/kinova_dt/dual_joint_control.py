#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import cv2
import numpy as np
import time

# === CONFIG ===
JOINT_NAMES_SIM = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
JOINT_NAMES_REAL = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
INCREMENT = 0.5

class DualPublisher(Node):
    def __init__(self):
        super().__init__('dual_joint_command_node')
        self.pub_sim = self.create_publisher(JointState, '/joint_command', 10)
        self.pub_real = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        # Subscribers for feedback
        self.sub_sim = self.create_subscription(JointState, '/joint_command', self.sim_callback, 10)
        self.sub_real = self.create_subscription(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', self.real_callback, 10)

        self.joints = [0.0] * 6  # Target positions (control only first 6 joints)
        self.time_sec = 0.0

        self.last_sent_time = 0  # For green button feedback

        # Feedback variables
        self.sim_feedback = [0.0] * 6
        self.real_feedback = [0.0] * 7

    def sim_callback(self, msg: JointState):
        if len(msg.position) >= 6:
            self.sim_feedback = list(msg.position[:6])

    def real_callback(self, msg: JointTrajectory):
        if msg.points and len(msg.points[0].positions) >= 7:
            self.real_feedback = list(msg.points[0].positions[:7])

    def publish_once(self):
        now = self.get_clock().now().to_msg()

        # --- Simulated robot message ---
        msg_sim = JointState()
        msg_sim.header.stamp = now
        msg_sim.name = JOINT_NAMES_SIM
        msg_sim.position = self.joints
        msg_sim.velocity = [0.0] * 6
        msg_sim.effort = []
        self.pub_sim.publish(msg_sim)
        self.get_logger().info(f"Published to sim: {self.joints}")

        # --- Real robot message ---
        msg_real = JointTrajectory()
        msg_real.header.stamp = now
        msg_real.joint_names = JOINT_NAMES_REAL

        point = JointTrajectoryPoint()
        point.positions = self.joints + [0.0]  # Joint 7 always 0.0
        point.time_from_start.sec = int(self.time_sec)
        point.time_from_start.nanosec = int((self.time_sec - int(self.time_sec)) * 1e9)

        msg_real.points.append(point)
        self.pub_real.publish(msg_real)
        self.get_logger().info(f"Published to real: {self.joints + [0.0]} time={self.time_sec}")

        # Green button visual feedback
        self.last_sent_time = time.time()


# === UI Helper ===
class OpenCVUI:
    def __init__(self, node: DualPublisher):
        self.node = node
        self.window = "Joint Control"
        self.buttons = []
        self.build_buttons()
        cv2.namedWindow(self.window)
        cv2.setMouseCallback(self.window, self.mouse_callback)

    def build_buttons(self):
        y_start = 60
        for i in range(6):
            self.buttons.append({"label": f"J{i+1}-", "x": 50, "y": y_start, "w": 60, "h": 35, "type": "dec", "joint": i})
            self.buttons.append({"label": f"J{i+1}+", "x": 150, "y": y_start, "w": 60, "h": 35, "type": "inc", "joint": i})
            y_start += 60

        # Time buttons
        self.buttons.append({"label": "Time-", "x": 50, "y": y_start, "w": 60, "h": 35, "type": "dec_time"})
        self.buttons.append({"label": "Time+", "x": 150, "y": y_start, "w": 60, "h": 35, "type": "inc_time"})
        y_start += 60

        # Send button
        self.buttons.append({"label": "SEND", "x": 50, "y": y_start, "w": 160, "h": 50, "type": "send"})

    def draw_ui(self):
        img = np.ones((700, 450, 3), dtype=np.uint8) * 50

        # Target joint values
        y_text = 85
        for i, val in enumerate(self.node.joints):
            cv2.putText(img, f"Target J{i+1}: {val:.2f}", (250, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            y_text += 60

        # Time value
        cv2.putText(img, f"Target Time: {self.node.time_sec:.2f}s", (250, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

        # Sim feedback
        y_feedback = 85
        for i, val in enumerate(self.node.sim_feedback):
            cv2.putText(img, f"Sim J{i+1}: {val:.2f}", (420, y_feedback), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100,255,100), 2)
            y_feedback += 60

        # Real feedback
        y_feedback = 85
        for i, val in enumerate(self.node.real_feedback):
            cv2.putText(img, f"Real J{i+1}: {val:.2f}", (570, y_feedback), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100,200,255), 2)
            y_feedback += 60

        # Draw buttons
        for btn in self.buttons:
            color = (200, 200, 200)
            if btn["type"] == "send" and (time.time() - self.node.last_sent_time) < 0.5:
                color = (0, 255, 0)  # Green when just sent
            cv2.rectangle(img, (btn["x"], btn["y"]), (btn["x"]+btn["w"], btn["y"]+btn["h"]), color, -1)
            cv2.putText(img, btn["label"], (btn["x"]+5, btn["y"]+22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)

        cv2.imshow(self.window, img)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            for btn in self.buttons:
                if btn["x"] <= x <= btn["x"]+btn["w"] and btn["y"] <= y <= btn["y"]+btn["h"]:
                    if btn["type"] == "inc":
                        self.node.joints[btn["joint"]] += INCREMENT
                    elif btn["type"] == "dec":
                        self.node.joints[btn["joint"]] -= INCREMENT
                    elif btn["type"] == "inc_time":
                        self.node.time_sec += INCREMENT
                    elif btn["type"] == "dec_time":
                        self.node.time_sec = max(0.0, self.node.time_sec - INCREMENT)
                    elif btn["type"] == "send":
                        self.node.publish_once()


def main(args=None):
    rclpy.init(args=args)
    node = DualPublisher()
    ui = OpenCVUI(node)

    try:
        while rclpy.ok():
            ui.draw_ui()
            key = cv2.waitKey(50)
            if key == 27:  # ESC to exit
                break
            rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
