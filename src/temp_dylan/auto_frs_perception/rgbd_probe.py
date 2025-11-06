#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration

class Probe(Node):
    def __init__(self):
        super().__init__('rgbd_probe')
        self.buf = Buffer(); self.tf = TransformListener(self.buf, self)
        self.sub = self.create_subscription(PointCloud2, '/camera/depth/points', self.cb, 10)

    def cb(self, msg: PointCloud2):
        # center pixel
        u0 = int(msg.width // 2); v0 = int(msg.height // 2)

        # Helper to try a pixel (u,v) and return first valid xyz
        def try_uv(u, v):
            # NOTE: Humble wants a *flattened* list here, not [(u,v)]
            for x, y, z in pc2.read_points(msg, field_names=('x','y','z'),
                                           skip_nans=False, uvs=[u, v]):
                if x == x and y == y and z == z:  # not NaN
                    return x, y, z
            return None

        xyz = try_uv(u0, v0)
        if xyz is None:
            # look around center in a small cross pattern
            for d in (1, 2, 3, 4, 6, 8):
                for du, dv in ((d,0),(-d,0),(0,d),(0,-d)):
                    u, v = u0+du, v0+dv
                    if 0 <= u < msg.width and 0 <= v < msg.height:
                        xyz = try_uv(u, v)
                        if xyz is not None:
                            break
                if xyz is not None:
                    break

        if xyz is None:
            self.get_logger().warn('No valid depth at/near center (likely sky/foliage).')
            return

        x, y, z = (float(v) for v in xyz)
        self.get_logger().info(f'Center @ {msg.header.frame_id}: ({x:.2f}, {y:.2f}, {z:.2f}) m')

        ps = PointStamped()
        ps.header = msg.header
        ps.point.x, ps.point.y, ps.point.z = x, y, z

        # Try map first; if not available, try odom
        for target in ('map', 'odom'):
            try:
                out = self.buf.transform(ps, target, timeout=Duration(seconds=0.3))
                self.get_logger().info(f'Center @ {target}: ({out.point.x:.2f}, {out.point.y:.2f}, {out.point.z:.2f}) m')
                break
            except Exception as e:
                pass

def main():
    rclpy.init()
    rclpy.spin(Probe())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
