#!/usr/bin/env python3
import math
import rospy
import tf2_ros
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LeftWallFollower:
    def __init__(self):
        rospy.init_node("left_wall_following")

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_cb)

        # TF listener (kept for lab requirement)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.scan = None
        self.rate = rospy.Rate(10)

        # ---- Tunables ----
        self.v_fwd = 0.14
        self.v_search = 0.08
        self.w_slight = 0.28

        # “good” left distance band
        self.wall_min = 0.10  # 10 cm
        self.wall_max = 0.20  # 20 cm

        # front obstacle thresholds (hysteresis)
        self.front_block = 0.30   # if below -> start turning right
        self.front_clear = 0.40   # if above -> go back to wall-follow

        # angles (radians)
        self.left_ang_min = 1.52
        self.left_ang_max = 1.62

        # left_limit is 7cm left of base_scan
        self.left_limit_y_offset = 0.07

        # define a small front cone around angle 0
        self.front_cone = 0.15  # ~8.6 degrees

        # simple FSM
        self.mode = "FOLLOW"  # FOLLOW or TURN_RIGHT

    def scan_cb(self, msg: LaserScan):
        self.scan = msg

    def _valid_range(self, r):
        if r is None or math.isinf(r) or math.isnan(r):
            return False
        if self.scan is not None:
            if r < (self.scan.range_min + 1e-3):
                return False
        return r > 0.0

    def tf_check(self):
        try:
            _ = self.tf_buffer.lookup_transform("base_scan", "left_limit",
                                                rospy.Time(0), rospy.Duration(0.05))
            return True
        except Exception:
            return False

    def get_front_distance(self):
        """Min valid distance in a small cone around 0 rad."""
        if self.scan is None:
            return None

        best = None
        a0 = self.scan.angle_min
        inc = self.scan.angle_increment

        for i, r in enumerate(self.scan.ranges):
            ang = a0 + i * inc
            if abs(ang) <= self.front_cone:
                if self._valid_range(r):
                    best = r if best is None else min(best, r)
        return best

    def compute_left_min_dist_in_left_limit(self):
        """Min distance of left-side rays, measured from left_limit origin."""
        if self.scan is None:
            return None

        min_dist = None
        a0 = self.scan.angle_min
        inc = self.scan.angle_increment

        for i, r in enumerate(self.scan.ranges):
            ang = a0 + i * inc
            if ang < self.left_ang_min or ang > self.left_ang_max:
                continue
            if not self._valid_range(r):
                continue

            x = r * math.cos(ang)
            y = r * math.sin(ang)

            # shift into left_limit frame
            y_ll = y - self.left_limit_y_offset
            d = math.sqrt(x * x + y_ll * y_ll)

            min_dist = d if min_dist is None else min(min_dist, d)

        return min_dist

    def run(self):
        while not rospy.is_shutdown():
            if self.scan is None:
                self.rate.sleep()
                continue

            _ = self.tf_check()

            front = self.get_front_distance()
            left_min = self.compute_left_min_dist_in_left_limit()

            vel = Twist()

            # ----- FSM: if front blocked, TURN RIGHT instead of stopping -----
            if self.mode == "FOLLOW":
                if front is not None and front < self.front_block:
                    self.mode = "TURN_RIGHT"

            elif self.mode == "TURN_RIGHT":
                if front is not None and front > self.front_clear:
                    self.mode = "FOLLOW"

            # ----- Action for each mode -----
            if self.mode == "TURN_RIGHT":
                # rotate right in place until the corridor opens up
                vel.linear.x = 0.0
                vel.angular.z = -0.55
                self.cmd_pub.publish(vel)
                self.rate.sleep()
                continue

            # ----- FOLLOW mode (left wall rules) -----
            if left_min is None:
                vel.linear.x = self.v_search
                vel.angular.z = +self.w_slight
            elif self.wall_min <= left_min <= self.wall_max:
                vel.linear.x = self.v_fwd
                vel.angular.z = 0.0
            elif left_min < self.wall_min:
                vel.linear.x = self.v_fwd
                vel.angular.z = -self.w_slight
            else:  # left_min > wall_max
                vel.linear.x = self.v_fwd
                vel.angular.z = +self.w_slight

            self.cmd_pub.publish(vel)
            self.rate.sleep()


if __name__ == "__main__":
    LeftWallFollower().run()
