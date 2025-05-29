## gesture_follow_turner/gesture_detector_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyzed.sl as sl
import cv2
import numpy as np

class GestureDetectorNode(Node):
    def __init__(self):
        super().__init__('gesture_detector')
        self.publisher_ = self.create_publisher(String, 'gesture_direction', 10)
        self.get_logger().info('Initializing ZED camera...')

        # Initialize ZED camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.coordinate_units = sl.UNIT.METER

        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"ZED camera failed to open: {status}")
            exit(1)

        self.runtime_parameters = sl.RuntimeParameters()
        self.image = sl.Mat()
        self.timer = self.create_timer(0.1, self.detect_gesture)
        self.get_logger().info('Gesture Detector Node with ZED has started.')

    def detect_gesture(self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            frame = self.image.get_data()
            # Convert to OpenCV format
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)

            # Stub: Replace with actual gesture detection
            direction = self.simulate_gesture(frame)

            msg = String()
            msg.data = direction
            self.publisher_.publish(msg)
            self.get_logger().info(f'Detected gesture: {direction}')

    def simulate_gesture(self, frame):
       
        return np.random.choice(['left', 'right', 'center'])

    def destroy_node(self):
        self.zed.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


## gesture_follow_turner/robot_turn_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math

class RobotTurnNode(Node):
    def __init__(self):
        super().__init__('robot_turner')
        self.subscriber_ = self.create_subscription(String, 'gesture_direction', self.turn_callback, 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_barrier_function)
        self.obstacle_distance = float('inf')
        self.current_direction = 'center'
        self.get_logger().info('Robot Turn Node with CBF has started.')

    def turn_callback(self, msg):
        self.current_direction = msg.data

    def control_barrier_function(self):
       
        self.obstacle_distance = self.get_simulated_obstacle_distance()

        if self.obstacle_distance < 1.0:
            self.get_logger().warn('Human too close. Stopping robot.')
            twist = Twist()
            self.publisher_.publish(twist)
            return

        twist = Twist()
        if self.current_direction == 'left':
            twist.angular.z = 0.5
        elif self.current_direction == 'right':
            twist.angular.z = -0.5
        else:
            twist.angular.z = 0.0

        self.publisher_.publish(twist)
        self.get_logger().info(f'Moving in response to: {self.current_direction}')

    def get_simulated_obstacle_distance(self):
        # TODO: Integrate with actual ZED depth map
        return math.inf


def main(args=None):
    rclpy.init(args=args)
    node = RobotTurnNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


## gesture_follow_turner/launch/gesture_follow_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gesture_follow_turner',
            executable='gesture_detector_node',
            name='gesture_detector'
        ),
        Node(
            package='gesture_follow_turner',
            executable='robot_turn_node',
            name='robot_turner'
        )
    ])
