import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np


class CameraPerceptionNode(Node):
    def __init__(self):
        super().__init__('camera_perception_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.publisher = self.create_publisher(String, '/world_state', 10)

        self.color_ranges = {
            'red_cube': [([0, 150, 50], [10, 255, 255]), ([170, 150, 50], [180, 255, 255])],
            'yellow_cube': [([20, 120, 100], [35, 255, 255])],
            'blue_cube': [([100, 150, 50], [130, 255, 255])],
            'green_cube': [([40, 100, 50], [80, 255, 255])],
            'purple_cube': [([135, 100, 50], [165, 255, 255])]
        }

        self.zones = {
            'zone_a': [0.54, -0.24, 0.50],
            'zone_b': [0.54, 0.0, 0.50],
            'zone_c': [0.54, 0.24, 0.50]
        }

        self.candidate_temp_slots = [
            [0.26, 0.0, 0.50],
            [0.26, -0.22, 0.50],
            [0.26, 0.22, 0.50]
        ]

    def pixel_to_world(self, u, v):
        scale = 0.00135
        x = 0.44 - (v - 240) * scale
        y = 0.00 - (u - 320) * scale
        return float(round(x, 3)), float(round(y, 3))

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detected_objects = {}

        for obj_name, ranges in self.color_ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 80:
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        u = int(M["m10"] / M["m00"])
                        v = int(M["m01"] / M["m00"])
                        wx, wy = self.pixel_to_world(u, v)
                        detected_objects[obj_name] = [wx, wy, 0.50]

        zone_occupancy = {}
        for z_name, z_pos in self.zones.items():
            occupier = None
            for o_name, o_pos in detected_objects.items():
                if np.hypot(z_pos[0] - o_pos[0], z_pos[1] - o_pos[1]) < 0.07:
                    occupier = o_name
                    break
            zone_occupancy[z_name] = occupier

        free_temp_position = None
        for slot in self.candidate_temp_slots:
            occupied = any(
                np.hypot(slot[0] - pos[0], slot[1] - pos[1]) < 0.07
                for pos in detected_objects.values()
            )
            if not occupied:
                free_temp_position = slot
                break

        world_state = {
            "objects": detected_objects,
            "zones": self.zones,
            "zone_occupancy": zone_occupancy,
            "temporary_position": free_temp_position
        }

        out_msg = String()
        out_msg.data = json.dumps(world_state)
        self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()