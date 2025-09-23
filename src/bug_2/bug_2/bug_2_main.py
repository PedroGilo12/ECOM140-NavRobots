import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import String
import math
import argparse

class Bug2Controller(Node):
    def __init__(self, goal_x, goal_y):
        super().__init__('bug2_controller')

        # -------------------- Subs --------------------
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # -------------------- Pubs --------------------
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.goal_marker_publisher = self.create_publisher(Marker, '/goal_marker', 10)
        self.mline_marker_publisher = self.create_publisher(Marker, '/tangent_marker', 10)
        self.state_pub = self.create_publisher(String, '/bug_state', 10)  # novo publisher
        self.last_state = None  # para não publicar repetido

        # -------------------- Goal --------------------
        self.goal_x = goal_x
        self.goal_y = goal_y

        # -------------------- Estados --------------------
        self.MODE_TO_GOAL = 0
        self.MODE_WALL_FOLLOWING = 1
        self.mode = self.MODE_TO_GOAL

        # -------------------- Pose --------------------
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.yaw = 0.0

        # -------------------- Leituras LIDAR --------------------
        self.dist_left = float('inf')
        self.dist_right = float('inf')
        self.dist_front = float('inf')
        self.dist_front_right = float('inf')

        # -------------------- Bug2 --------------------
        self.hit_point = None  

        # -------------------- Parâmetros --------------------
        self.distance_goal_tolerance = 0.2
        self.obstacle_detect_threshold = 0.4
        self.max_linear_speed = 0.2
        self.max_angular_speed = 1.0
        self.kp_ang = 1.5
        self.kp_lin = 0.5

        # Controle de parede
        self.desired_wall_dist = 0.4
        self.k_dist = 1.2
        self.k_angle = 0.8

        # Ponto inicial
        self.start_x = None
        self.start_y = None
        self.start_point_set = False

        # Debug / throttle logs
        self.debug_counter = 0
        self.debug_log_every = 5  # log a cada N ciclos (timer=0.1s => 5 -> 0.5s)

        # Timer principal
        self.timer = self.create_timer(0.1, self.control_loop)

    # --------------------- Helpers LIDAR ---------------------
    def _angle_to_index(self, angle, angle_min, angle_inc, ranges_len):
        rel = angle - angle_min
        idx = int(round(rel / angle_inc))
        return idx % ranges_len

    def _min_in_sector(self, ranges, angle_min, angle_inc, start_deg, end_deg):
        start_rad = math.radians(start_deg)
        end_rad = math.radians(end_deg)
        n = len(ranges)

        s_idx = self._angle_to_index(start_rad, angle_min, angle_inc, n)
        e_idx = self._angle_to_index(end_rad, angle_min, angle_inc, n)

        if s_idx <= e_idx:
            sector = ranges[s_idx:e_idx+1]
        else:
            sector = list(ranges[s_idx:]) + list(ranges[:e_idx+1])

        valid = [r for r in sector if (r and math.isfinite(r) and r > 0.0)]
        return min(valid) if valid else float('inf')

    # --------------------- Scan Callback ---------------------
    def scan_callback(self, msg: LaserScan):
        ranges = list(msg.ranges)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        try:
            # Leituras originais
            self.dist_left = self._min_in_sector(ranges, angle_min, angle_inc, 80, 100)
            self.dist_right = self._min_in_sector(ranges, angle_min, angle_inc, -100, -80)

            # Leituras para wall following
            self.dist_front = self._min_in_sector(ranges, angle_min, angle_inc, -30, 30)
            self.dist_front_right = self._min_in_sector(ranges, angle_min, angle_inc, -100, -80)

        except Exception as e:
            self.get_logger().warn(f"Erro em scan_callback: {e}")
            self.dist_front = self.dist_front_right = float('inf')

    # --------------------- Odom Callback ---------------------
    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        if not self.start_point_set:
            self.start_x = self.robot_x
            self.start_y = self.robot_y
            self.start_point_set = True
            self.get_logger().info(f"Ponto inicial: ({self.start_x:.2f}, {self.start_y:.2f})")

    # --------------------- Marker ---------------------
    def publish_goal_marker(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal_namespace"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.goal_x)
        marker.pose.position.y = float(self.goal_y)
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime.sec = 5
        self.goal_marker_publisher.publish(marker)

    # --------------------- Marker M-line ---------------------
    def publish_mline(self):
        if not self.start_point_set:
            return

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mline_namespace"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.points = [
            self._make_point(self.start_x, self.start_y),
            self._make_point(self.goal_x, self.goal_y)
        ]

        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.mline_marker_publisher.publish(marker)

    # --------------------- Helper ---------------------
    def _make_point(self, x, y, z=0.0):
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    # --------------------- Loop principal ---------------------
    def control_loop(self):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_link"

        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        distance_to_goal = math.hypot(dx, dy)

        # publicações de debug
        self.publish_goal_marker()
        self.publish_mline()
        self.debug_counter += 1

        if distance_to_goal < self.distance_goal_tolerance:
            self.get_logger().info("Chegou no goal! ✅")
            self.cmd_pub.publish(TwistStamped())
            self.publish_state()  # publica estado final
            return

        if self.mode == self.MODE_TO_GOAL:
            self.move_to_goal(twist_stamped, distance_to_goal)
        elif self.mode == self.MODE_WALL_FOLLOWING:
            self.follow_wall(twist_stamped, distance_to_goal)

        # Saturação
        twist_stamped.twist.linear.x = max(min(twist_stamped.twist.linear.x, self.max_linear_speed), -self.max_linear_speed)
        twist_stamped.twist.angular.z = max(min(twist_stamped.twist.angular.z, self.max_angular_speed), -self.max_angular_speed)

        self.cmd_pub.publish(twist_stamped)
        self.publish_state()  # publica estado atual

        if self.debug_counter >= self.debug_log_every:
            self.debug_counter = 0
            self._log_debug_state(twist_stamped, distance_to_goal)

    # --------------------- Go to goal ---------------------
    def move_to_goal(self, twist_stamped: TwistStamped, distance_to_goal: float):
        target_theta = math.atan2(self.goal_y - self.robot_y, self.goal_x - self.robot_x)
        angle_diff = math.atan2(math.sin(target_theta - self.yaw), math.cos(target_theta - self.yaw))

        if self.dist_front < self.obstacle_detect_threshold:
            self.get_logger().info("Obstáculo na frente → mudando para WALL FOLLOWING")
            self.mode = self.MODE_WALL_FOLLOWING
            self.hit_point = (self.robot_x, self.robot_y)
            return

        twist_stamped.twist.angular.z = self.kp_ang * angle_diff
        twist_stamped.twist.linear.x = 0.0 if abs(angle_diff) > 0.7 else min(self.kp_lin * distance_to_goal, self.max_linear_speed)

    # --------------------- Wall following ---------------------
    def follow_wall(self, twist_stamped: TwistStamped, distance_to_goal: float):
        front_blocked = self.dist_front < self.obstacle_detect_threshold
        front_right_has_wall = self.dist_front_right < 0.6

        if front_blocked:
            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.angular.z = 0.6

        elif front_right_has_wall and front_blocked:
            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.angular.z = 0.6

        elif front_right_has_wall:
            twist_stamped.twist.linear.x = 0.15
            twist_stamped.twist.angular.z = 0.0
        else:
            twist_stamped.twist.linear.x = 0.5
            twist_stamped.twist.angular.z = -0.8

        if self.is_on_mline() and self.hit_point is not None:
            dist_now = math.hypot(self.robot_x - self.goal_x, self.robot_y - self.goal_y)
            dist_hit = math.hypot(self.hit_point[0] - self.goal_x, self.hit_point[1] - self.goal_y)
            if dist_now < dist_hit:
                self.mode = self.MODE_TO_GOAL

    # --------------------- M-line ---------------------
    def is_on_mline(self, threshold=0.2):
        if self.start_x is None or self.start_y is None:
            return False

        x1, y1 = self.start_x, self.start_y
        x2, y2 = self.goal_x, self.goal_y

        if x1 == x2 and y1 == y2:
            return True

        num = abs((y2 - y1)*self.robot_x - (x2 - x1)*self.robot_y + x2*y1 - y2*x1)
        den = math.hypot(y2 - y1, x2 - x1)
        dist = num / den if den > 1e-6 else float('inf')
        return dist < threshold

    # --------------------- Publicar estado ---------------------
    def publish_state(self):
        mode_str = "TO_GOAL" if self.mode == self.MODE_TO_GOAL else "WALL_FOLLOW"
        if mode_str != self.last_state:
            msg = String()
            msg.data = mode_str
            self.state_pub.publish(msg)
            self.get_logger().info(f"Bug state → {mode_str}")
            self.last_state = mode_str

    # --------------------- Debug ---------------------
    def _log_debug_state(self, twist_stamped: TwistStamped, distance_to_goal: float):
        mode_str = "TO_GOAL" if self.mode == self.MODE_TO_GOAL else "WALL_FOLLOW"
        hit_str = f"{self.hit_point[0]:.3f},{self.hit_point[1]:.3f}" if self.hit_point else "None"
        debug_msg = (
            f"[{mode_str}] pos=({self.robot_x:.3f},{self.robot_y:.3f}) yaw={self.yaw:.3f} "
            f"goal=({self.goal_x:.3f},{self.goal_y:.3f}) d_goal={distance_to_goal:.3f} "
            f"cmd_lin={twist_stamped.twist.linear.x:.3f} cmd_ang={twist_stamped.twist.angular.z:.3f} hit={hit_str}"
        )
        self.get_logger().info(debug_msg)

# --------------------- Main ---------------------
def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal_x", type=float, required=True)
    parser.add_argument("--goal_y", type=float, required=True)
    parsed_args, _ = parser.parse_known_args()

    node = Bug2Controller(parsed_args.goal_x, parsed_args.goal_y)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
