#!/usr/bin/env python3
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tensorflow.keras.models import load_model
from ament_index_python.packages import get_package_share_directory

# --- Locate models via package share ---
pkg_share = get_package_share_directory('auto_frs_perception')
models_dir = os.path.join(pkg_share, 'models')

model_path = os.path.join(models_dir, 'tm_model.h5')
labels_path = os.path.join(models_dir, 'tm_labels.txt')


class TestSub(Node):
    def __init__(self):
        super().__init__('test_sub')

        # --- Load Teachable Machine model ---
        self.model = load_model(model_path, compile=False)
        with open(labels_path, 'r') as f:
            self.class_names = [line.strip().split(" ", 1)[-1] for line in f]

        self.input_size = (224, 224)  # Teachable Machine default
        self.conf_thresh = 0.7        # confidence threshold

        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)

        self.get_logger().info("Camera + TM classifier started!")

    def preprocess(self, cv_image):
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, self.input_size)
        img = img.astype(np.float32) / 255.0
        return np.expand_dims(img, axis=0)

    def image_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run inference
        inp = self.preprocess(cv_image)
        preds = self.model.predict(inp, verbose=0)[0]

        top_idx = int(np.argmax(preds))
        top_conf = float(preds[top_idx])
        top_label = self.class_names[top_idx]

        # Overlay prediction on live feed
        text = f"{top_label} ({top_conf:.2f})"
        cv2.putText(cv_image, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("Camera + TM Classifier", cv_image)
        cv2.waitKey(1)

        if top_conf >= self.conf_thresh:
            self.get_logger().info(f"Detected: {top_label} ({top_conf:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = TestSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
