#!/usr/bin/env python3
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point


def best_effort(depth=10):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class ObjectMapper(Node):
    def __init__(self):
        super().__init__('object_mapper')

        # --- QoS matches your camera topics ---
        self.image_qos = best_effort()
        self.cloud_qos = best_effort()

        # --- I/O ---
        self.bridge = CvBridge()
        self.sub_img   = self.create_subscription(Image, '/camera/image',
                                          self._on_image, self.image_qos)
        self.sub_cloud = self.create_subscription(PointCloud2, '/camera/depth/points',
                                          self._on_cloud, self.cloud_qos)

        self.pub_point  = self.create_publisher(PointStamped, '/detected_objects', 10)
        self.pub_marker = self.create_publisher(Marker, '/marker', 10)

        # TF2 (map <- camera frame)
        self.tfbuf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tfl   = tf2_ros.TransformListener(self.tfbuf, self)

        # State
        self.last_cloud = None      # latest PointCloud2
        self.last_cloud_frame = None
        self.declare_parameter('debug_view', True)
        self.debug_view = bool(self.get_parameter('debug_view').value)

        self.get_logger().info('ObjectMapper ready (camera + depth + TF2)')

    # --------- Subscribers ----------
    def _on_cloud(self, msg: PointCloud2):
        self.last_cloud = msg
        self.last_cloud_frame = msg.header.frame_id

    def _on_image(self, msg: Image):
        # Need a cloud to get range/xyz
        if self.last_cloud is None:
            return

        # 1) BGR image
        cv_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 2) Threshold "pink" hotspot in HSV
        hsv = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2HSV)

        # two bands around pink/magenta (adjust if needed)
        lower1 = np.array([150,  30,  60], dtype=np.uint8)
        upper1 = np.array([179, 255, 255], dtype=np.uint8)
        lower2 = np.array([140,  30,  60], dtype=np.uint8)
        upper2 = np.array([150, 255, 255], dtype=np.uint8)


        # clean up mask
        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7,7), np.uint8))

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            if self.debug_view:
                self._show_debug(cv_bgr, mask, None)
            return

        # 3) pick largest blob
        cnt = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        if area < 80:  # ignore tiny specks
            if self.debug_view:
                self._show_debug(cv_bgr, mask, None)
            return

        M = cv2.moments(cnt)
        if M['m00'] == 0:
            if self.debug_view:
                self._show_debug(cv_bgr, mask, None)
            return
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        # 4) Look up XYZ at that pixel from organized point cloud
        xyz = self._xyz_from_cloud(self.last_cloud, cx, cy)
        if xyz is None or any(np.isnan(xyz)):
            if self.debug_view:
                self._show_debug(cv_bgr, mask, (cx, cy))
            return

        x_c, y_c, z_c = xyz  # camera frame

        # 5) Transform to a global frame we actually have
        pt_cam = PointStamped()
        pt_cam.header = self.last_cloud.header            # usually camera_depth_optical_frame
        pt_cam.point.x, pt_cam.point.y, pt_cam.point.z = float(x_c), float(y_c), float(z_c)

        pt_out = None
        for target in ('map', 'odom'):
            try:
                tf = self.tfbuf.lookup_transform(target, pt_cam.header.frame_id, rclpy.time.Time())
                pt_out = do_transform_point(pt_cam, tf)
                pt_out.header.frame_id = target
                break
            except TransformException as e:
                self.get_logger().warn(f'TF lookup to {target} failed: {e}')

# Last resort: publish in camera frame so we can still see something
        if pt_out is None:
            pt_out = pt_cam
            self.get_logger().warn('Publishing in camera frame (no map/odom TF chain available).')

# 6) Publish point + RViz marker
        self.pub_point.publish(pt_out)
        self._publish_marker(pt_out)
        self.get_logger().info(
            f"Hotspot @ {pt_out.header.frame_id}: "
            f"({pt_out.point.x:.2f}, {pt_out.point.y:.2f}, {pt_out.point.z:.2f})"
)

        if self.debug_view:
            self._show_debug(cv_bgr, mask, (cx, cy))

    # --------- Helpers ----------
    def _xyz_from_cloud(self, cloud: PointCloud2, u: int, v: int):
        """Return (x,y,z) at pixel (u,v) from an organized PointCloud2."""
        if cloud.width == 0 or cloud.height == 0:
            return None
        if not (0 <= u < cloud.width and 0 <= v < cloud.height):
            return None
        # read_points with uvs uses the cloud’s organization
        gen = pc2.read_points(cloud, field_names=('x', 'y', 'z'),
                              skip_nans=False, uvs=[(u, v)])
        try:
            x, y, z = next(gen)
            return np.array([x, y, z], dtype=float)
        except StopIteration:
            return None

    def _publish_marker(self, pt_map: PointStamped):
        mk = Marker()
        mk.header.frame_id = 'map'
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns = 'hotspots'
        mk.id = 0
        mk.type = Marker.SPHERE
        mk.action = Marker.ADD
        mk.pose.position.x = pt_map.point.x
        mk.pose.position.y = pt_map.point.y
        mk.pose.position.z = pt_map.point.z
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = mk.scale.z = 0.4
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.2, 0.7, 0.9
        self.pub_marker.publish(mk)

    def _show_debug(self, bgr, mask, center):
        dbg = bgr.copy()
        if center is not None:
            cv2.circle(dbg, center, 8, (0, 255, 0), 2)
        cv2.imshow('Camera Feed + Hotspot', dbg)
        cv2.imshow('Hotspot mask', mask)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = ObjectMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
