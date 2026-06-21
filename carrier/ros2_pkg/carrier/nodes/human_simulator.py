#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class HumanSimulator(Node):
    def __init__(self):
        super().__init__('human_simulator')
        self._pub_odom = self.create_publisher(Odometry, '/human/odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.1, self._timer_cb)
        
        # Start far from scout (e.g., origin) and walk in a circle or simple path
        self._x = -2.0
        self._y = -2.0
        self._yaw = 0.0
        self._speed = 0.3 # 0.3 m/s human walk speed
        self._start_time = time.time()
        self.get_logger().info("Simulating human UWB tag walking around...")

    def _timer_cb(self):
        t = time.time() - self._start_time
        
        # Simple Eight-figure or circle path
        # x = 4*sin(t/5) - 2
        # y = 4*sin(t/5)*cos(t/5) - 2
        # Let's just do a circle
        self._x = 4.0 * math.cos(t * 0.2) - 4.0
        self._y = 4.0 * math.sin(t * 0.2)
        
        dx = -0.8 * math.sin(t * 0.2)
        dy = 0.8 * math.cos(t * 0.2)
        self._yaw = math.atan2(dy, dx)
        
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "scout/map" # Assuming shared map
        msg.child_frame_id = "human_link"
        
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        
        # Quat from yaw
        q = self._quaternion_from_euler(0, 0, self._yaw)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        
        self._pub_odom.publish(msg)

        # Broadcast TF for visualization
        tfs = TransformStamped()
        tfs.header.stamp = msg.header.stamp
        tfs.header.frame_id = "scout/map"
        tfs.child_frame_id = "human_link"
        tfs.transform.translation.x = self._x
        tfs.transform.translation.y = self._y
        tfs.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(tfs)

    @staticmethod
    def _quaternion_from_euler(ai, aj, ak):
        ai /= 2.0; aj /= 2.0; ak /= 2.0
        ci = math.cos(ai); si = math.sin(ai)
        cj = math.cos(aj); sj = math.sin(aj)
        ck = math.cos(ak); sk = math.sin(ak)
        cc = ci*ck; cs = ci*sk; sc = si*ck; ss = si*sk
        return [cj*sc - sj*cs, cj*ss + sj*cc, cj*cs - sj*sc, cj*cc + sj*ss]

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(HumanSimulator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
