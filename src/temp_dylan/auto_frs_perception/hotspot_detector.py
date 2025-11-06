#!/usr/bin/env python3
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2

from tensorflow.keras.models import load_model
from ament_index_python.packages import get_package_share_directory


class HotspotDetector(Node):
    def __init__(self):
        super().__init__('hotspot_detector')

        # Locate model files
        pkg_share = get_package_share_directory('auto_frs_perception')
        models_dir = os.path.join(pkg_share, 'models')

        model_path = os.path.join(models_dir, 'keras_model.h5')
        labels_path = os.path.join(models_dir, 'labels.txt')

        # Load model + labels
        self.model = load_model(model_path, compile=False)
        with open(labels_path, 'r') as f:
            self.class_names = [line.strip().split(" ", 1)[-1] for line in f]

        self.get_logger().info(f"Loaded model with classes: {self.class_names}")

        # Preprocessing settings
        self.input_size = (224, 224)
        self.conf_thresh = 0.7

        # ROS I/O
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/perception/detections', 10)

        self._last_pub = 0.0
        self._pub_period = 0.5

    def preprocess(self, cv_image):
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, self.input_size)
        img = img.astype(np.float32) / 255.0
        return np.expand_dims(img, axis=0)

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        # Run inference
        inp = self.preprocess(cv_image)
        preds = self.model.predict(inp, verbose=0)[0]

        top_idx = int(np.argmax(preds))
        top_conf = float(preds[top_idx])
        top_label = self.class_names[top_idx]

        # Throttle publishing
        now = time.time()
        if now - self._last_pub < self._pub_period:
            return
        self._last_pub = now

        # Publish to RViz
        marker_array = MarkerArray()
        if top_conf >= self.conf_thresh:
            m = Marker()
            m.header.frame_id = "camera_link"
            m.header.stamp = msg.header.stamp
            m.ns = "detections"
            m.id = 0
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD

            m.pose.position.x = 1.0
            m.pose.position.y = 0.0
            m.pose.position.z = 0.0

            m.scale.z = 0.3
            m.color.r, m.color.g, m.color.b, m.color.a = (1.0, 1.0, 1.0, 1.0)
            m.text = f"{top_label} ({top_conf:.2f})"

            marker_array.markers.append(m)
            self.get_logger().info(f"Detected: {top_label} ({top_conf:.2f})")

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = HotspotDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
