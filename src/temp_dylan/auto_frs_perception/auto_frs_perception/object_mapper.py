#!/usr/bin/env python3
# ros2 run auto_frs_perception object_mapper --ros-args -p debug_view:=true

import math, time, struct
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped, PoseArray, Pose
from visualization_msgs.msg import Marker

import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point


def best_effort(depth=10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class ObjectMapper(Node):
    def __init__(self):
        super().__init__('object_mapper')

        # ---------- Tunables (ROS params) ----------
        self.declare_parameter('target_color', 'blue')     # kept for compatibility
        self.declare_parameter('sat_min', 80)
        self.declare_parameter('val_min', 120)
        self.declare_parameter('crop_top', 0.45)
        self.declare_parameter('area_min', 250)
        self.declare_parameter('circ_min', 0.60)
        self.declare_parameter('debug_view', True)
        # NEW: de-dup radius for “print once”
        self.declare_parameter('archive_radius_m', 0.60)

        # NEW: topic params (Husky-style defaults, override for drone)
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('depth_topic', '/camera/depth/points')
        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value

        self.target_color = str(self.get_parameter('target_color').value).lower()
        self.sat_min      = int(self.get_parameter('sat_min').value)
        self.val_min      = int(self.get_parameter('val_min').value)
        self.crop_top     = float(self.get_parameter('crop_top').value)
        self.area_min     = float(self.get_parameter('area_min').value)
        self.circ_min     = float(self.get_parameter('circ_min').value)
        self.debug_view   = bool(self.get_parameter('debug_view').value)
        self.archive_r    = float(self.get_parameter('archive_radius_m').value)

        # ---------- I/O ----------
        self.bridge = CvBridge()
        self.image_qos = best_effort()
        self.cloud_qos = best_effort()

        self.sub_img   = self.create_subscription(
            Image, image_topic, self._on_image, self.image_qos)
        self.sub_cloud = self.create_subscription(
            PointCloud2, depth_topic, self._on_cloud, self.cloud_qos)

        self.pub_point  = self.create_publisher(PointStamped, '/detected_objects', 10)
        self.pub_marker = self.create_publisher(Marker, '/marker', 10)

        # NEW: latched archive so other nodes can read all hotspots at any time
        archive_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub_archive = self.create_publisher(PoseArray, '/hotspots/archive', archive_qos)
        self.archive = PoseArray()                # will set frame_id on first insert
        self.archive.header.frame_id = ''

        # TF2
        self.tfbuf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tfl   = tf2_ros.TransformListener(self.tfbuf, self)

        # State
        self.last_cloud = None
        self.last_cloud_frame = None

        # Debug-rate limiter so the GUI stays responsive
        self._last_dbg = 0.0
        self._dbg_hz   = 5.0

        self.get_logger().info(
            f'ObjectMapper ready (image={image_topic}, depth={depth_topic}, TF2 enabled)'
        )

    # ---------- subscribers ----------
    def _on_cloud(self, msg: PointCloud2):
        self.last_cloud = msg
        self.last_cloud_frame = msg.header.frame_id

    def _on_image(self, msg: Image):
        if self.last_cloud is None:
            return

        # 1) Always get a BGR image (cv2 expects BGR)
        cv_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_rgb = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)

        # 2) Two complementary color tests
        hsv = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2HSV)

        # HSV red
        red1 = cv2.inRange(hsv, np.array([0,   60,  60], dtype=np.uint8),
                                 np.array([10, 255, 255], dtype=np.uint8))
        red2 = cv2.inRange(hsv, np.array([170, 60,  60], dtype=np.uint8),
                                 np.array([179, 255, 255], dtype=np.uint8))
        mask_hsv = cv2.bitwise_or(red1, red2)

        # Red-dominance in BGR
        b, g, r = cv2.split(cv_bgr)
        rg_max = cv2.max(b, g)
        rd = (r.astype(np.int16) - rg_max.astype(np.int16)) > 40
        rd &= (r > 120)
        mask_rd = (rd.astype(np.uint8) * 255)

        # Combined mask
        mask = cv2.bitwise_or(mask_hsv, mask_rd)

        # blob code
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        nonzero = int(cv2.countNonZero(mask))
        if nonzero == 0:
            if self.debug_view:
                self._show_debug(cv_bgr, mask, None)
            return

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            if self.debug_view:
                self._show_debug(cv_bgr, mask, None)
            return

        cnt = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        if area < 40:
            if self.debug_view:
                self._show_debug(cv_bgr, mask, None)
            return

        M = cv2.moments(cnt)
        if M['m00'] == 0:
            if self.debug_view:
                self._show_debug(cv_bgr, mask, None)
            return

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        # ----- depth lookup -----
        xyz = self._xyz_from_cloud(self.last_cloud, cx, cy)
        if xyz is None or any(np.isnan(xyz)):
            self._maybe_show(cv_rgb, mask, (cx, cy))
            return
        x_c, y_c, z_c = map(float, xyz)

        # ----- transform to a global-ish frame we actually have -----
        pt_cam = PointStamped()
        pt_cam.header = self.last_cloud.header
        pt_cam.point.x, pt_cam.point.y, pt_cam.point.z = x_c, y_c, z_c

        pt_out = None
        for target in ('map', 'odom', 'base_link'):
            try:
                tf = self.tfbuf.lookup_transform(
                    target, pt_cam.header.frame_id, rclpy.time.Time())
                pt_out = do_transform_point(pt_cam, tf)
                pt_out.header.frame_id = target
                break
            except TransformException:
                pass
        if pt_out is None:
            pt_out = pt_cam  # publish in camera frame as a last resort

        # ----- publish as before -----
        self.pub_point.publish(pt_out)
        self._publish_marker(pt_out)

        # ----- print once + store in archive -----
        if self._record_if_new(pt_out):
            self.get_logger().info(
                f"Hotspot @ {pt_out.header.frame_id}: "
                f"({pt_out.point.x:.2f}, {pt_out.point.y:.2f}, {pt_out.point.z:.2f})"
            )
            self.pub_archive.publish(self.archive)  # latched update

        self._maybe_show(cv_rgb, mask, (cx, cy))

    # ---------- helpers ----------
    def _xyz_from_cloud(self, cloud: PointCloud2, u: int, v: int):
        """Directly index an *organized* PointCloud2 at pixel (u,v)."""
        if cloud.width == 0 or cloud.height == 0:
            return None
        if not (0 <= u < cloud.width and 0 <= v < cloud.height):
            return None

        idx = v * cloud.row_step + u * cloud.point_step
        offs = {f.name: f.offset for f in cloud.fields}
        if not all(k in offs for k in ('x', 'y', 'z')):
            return None
        endian = '>' if cloud.is_bigendian else '<'

        try:
            x = struct.unpack_from(endian + 'f', cloud.data, idx + offs['x'])[0]
            y = struct.unpack_from(endian + 'f', cloud.data, idx + offs['y'])[0]
            z = struct.unpack_from(endian + 'f', cloud.data, idx + offs['z'])[0]
        except Exception:
            return None

        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            return None

        if (x == 0.0 and y == 0.0 and z == 0.0) or any(
            map(lambda q: not math.isfinite(q), (x, y, z))
        ):
            best = None
            for dv in (-1, 0, 1):
                for du in (-1, 0, 1):
                    uu, vv = u + du, v + dv
                    if uu < 0 or vv < 0 or uu >= cloud.width or vv >= cloud.height:
                        continue
                    ii = vv * cloud.row_step + uu * cloud.point_step
                    try:
                        xn = struct.unpack_from(endian + 'f', cloud.data, ii + offs['x'])[0]
                        yn = struct.unpack_from(endian + 'f', cloud.data, ii + offs['y'])[0]
                        zn = struct.unpack_from(endian + 'f', cloud.data, ii + offs['z'])[0]
                        if math.isfinite(xn) and math.isfinite(yn) and math.isfinite(zn):
                            best = (xn, yn, zn)
                            break
                    except Exception:
                        pass
                if best:
                    break
            if best:
                x, y, z = best
            else:
                return None

        return np.array([x, y, z], dtype=float)

    def _publish_marker(self, pt: PointStamped):
        mk = Marker()
        mk.header.frame_id = pt.header.frame_id
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns = 'hotspots'
        mk.id = 0
        mk.type = Marker.SPHERE
        mk.action = Marker.ADD
        mk.pose.position.x = pt.point.x
        mk.pose.position.y = pt.point.y
        mk.pose.position.z = pt.point.z
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = mk.scale.z = 0.4
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.2, 0.8, 1.0, 0.9
        self.pub_marker.publish(mk)

    # make previous `_show_debug` calls safe
    def _show_debug(self, rgb, mask, center):
        self._maybe_show(rgb, mask, center)

    def _maybe_show(self, rgb, mask, center):
        if not self.debug_view:
            return
        now = time.monotonic()
        if now - self._last_dbg < 1.0 / self._dbg_hz:
            return
        self._last_dbg = now
        dbg = rgb.copy()
        if center is not None:
            cv2.circle(dbg, center, 8, (0, 255, 0), 2)
        cv2.imshow('Camera Feed + Hotspot', cv2.cvtColor(dbg, cv2.COLOR_RGB2BGR))
        cv2.imshow('Hotspot mask', mask)
        cv2.waitKey(1)

    # store unique hotspots; print once
    def _record_if_new(self, pt: PointStamped) -> bool:
        # decide archive frame on first insert
        if not self.archive.header.frame_id:
            self.archive.header.frame_id = pt.header.frame_id

        # if this detection is in a different frame, try to transform to archive frame
        if pt.header.frame_id != self.archive.header.frame_id:
            try:
                tf = self.tfbuf.lookup_transform(
                    self.archive.header.frame_id,
                    pt.header.frame_id,
                    rclpy.time.Time()
                )
                pt = do_transform_point(pt, tf)
            except TransformException:
                return False  # can't compare → skip

        p = np.array([pt.point.x, pt.point.y, pt.point.z])
        for pose in self.archive.poses:
            q = np.array([pose.position.x, pose.position.y, pose.position.z])
            if np.linalg.norm(p - q) < self.archive_r:
                return False  # already have a nearby hotspot

        newp = Pose()
        newp.position.x, newp.position.y, newp.position.z = p.tolist()
        newp.orientation.w = 1.0
        self.archive.poses.append(newp)
        self.archive.header.stamp = self.get_clock().now().to_msg()
        return True


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
