import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Point, TwistStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import math
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import matplotlib.pyplot as plt
import random
import time

class PotentialFieldNavigator(Node):
    def __init__(self):
        super().__init__('potential_field_navigator')

        self.INITIAL_GOAL_WORLD = (-3, 3)
        self.MAP_IMAGE_PATH = 'mapa_rrt2.png'

        self.K_ATTR = 0.5
        self.K_REP = 100.0
        self.REP_RANGE = 100
        self.CONTROL_LOOP_PERIOD = 0.1
        self.GOAL_TOLERANCE = 0.15
        self.YAW_TOLERANCE = 0.1
        self.LINEAR_VEL = 0.25
        self.ANGULAR_VEL_GAIN = 1.8

        self.last_dist_to_goal = None
        self.last_progress_time = self._now_seconds()
        self.STUCK_TIME_THRESHOLD = 10
        self.PROGRESS_EPS = 0.02
        self.RECOVERY_DURATION = 20
        self.temporary_goal_active = False
        self.temporary_goal_world = None
        self.temporary_goal_start_time = None
        self.recent_recovery_time = 0.0
        self.MIN_TIME_BETWEEN_RECOVERIES = 2.0

        self.goal_sub = self.create_subscription(Point, 'goal', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/goal_marker', 10)

        self.position = None
        self.yaw = 0.0
        self.final_goal_world = None
        self.force_x = None
        self.force_y = None
        self.mapa_bin = None
        self.is_first_plot = True

        try:
            self.load_map(self.MAP_IMAGE_PATH)
            self.get_logger().info(f"Mapa '{self.MAP_IMAGE_PATH}' carregado com sucesso.")
        except FileNotFoundError:
            self.get_logger().error(f"Erro: imagem do mapa não encontrada em '{self.MAP_IMAGE_PATH}'")
            rclpy.shutdown()
            return

        self.MAP_WIDTH_METERS = 8.0
        self.MAP_HEIGHT_METERS = 8.0
        self.scale_x = self.MAP_WIDTH_METERS / self.MAP_WIDTH_PIXELS
        self.scale_y = self.MAP_HEIGHT_METERS / self.MAP_HEIGHT_PIXELS

        self.get_logger().info(f"Processando goal inicial: {self.INITIAL_GOAL_WORLD}")
        self.process_new_goal(self.INITIAL_GOAL_WORLD)

        self.timer = self.create_timer(self.CONTROL_LOOP_PERIOD, self.control_loop)
        self.get_logger().info("Navegação iniciada.")

    def _now_seconds(self):
        tmsg = self.get_clock().now().to_msg()
        return float(tmsg.sec) + float(tmsg.nanosec) * 1e-9

    def load_map(self, image_path):
        img = Image.open(image_path).convert('L')
        img_np = np.array(img)
        self.mapa_bin = (img_np > 127).astype(np.uint8)
        self.MAP_HEIGHT_PIXELS, self.MAP_WIDTH_PIXELS = self.mapa_bin.shape

    def pixel_to_world(self, x_pixel, y_pixel):
        x_world = (x_pixel - self.MAP_WIDTH_PIXELS / 2) * self.scale_x
        y_world = (self.MAP_HEIGHT_PIXELS / 2 - y_pixel) * self.scale_y
        return x_world, y_world

    def world_to_pixel(self, x_world, y_world):
        x_pixel = (x_world / self.scale_x) + (self.MAP_WIDTH_PIXELS / 2)
        y_pixel = (self.MAP_HEIGHT_PIXELS / 2) - (y_world / self.scale_y)
        x_pixel = int(np.clip(x_pixel, 0, self.MAP_WIDTH_PIXELS - 1))
        y_pixel = int(np.clip(y_pixel, 0, self.MAP_HEIGHT_PIXELS - 1))
        return x_pixel, y_pixel

    def goal_callback(self, msg: Point):
        self.process_new_goal((msg.x, msg.y))

    def process_new_goal(self, goal_world_coords):
        self.final_goal_world = goal_world_coords
        self.get_logger().info(f"Novo goal: x={goal_world_coords[0]:.2f}, y={goal_world_coords[1]:.2f}")

        goal_pixel_x, goal_pixel_y = self.world_to_pixel(*goal_world_coords)
        if self.mapa_bin[goal_pixel_y, goal_pixel_x] == 0:
            self.get_logger().warn("Goal dentro de obstáculo, cancelando navegação.")
            self.final_goal_world = None
            self.force_x = None
            self.force_y = None
            self.delete_debug_marker(self.goal_marker_pub, "goal_marker", 0)
            return

        U_total, X, Y = self.calculate_potential_field((goal_pixel_x, goal_pixel_y))
        self.publish_debug_marker(
            self.goal_marker_pub,
            goal_world_coords[0],
            goal_world_coords[1],
            r=1.0, g=0.0, b=0.0,
            ns="goal_marker",
            marker_id=0
        )

        if self.is_first_plot:
            self.plot_potential_field(U_total, X, Y, (goal_pixel_x, goal_pixel_y))
            self.is_first_plot = False

    def calculate_potential_field(self, goal_pixel):
        h, w = self.mapa_bin.shape
        Y, X = np.mgrid[0:h, 0:w]

        dist_goal = np.sqrt((X - goal_pixel[0])**2 + (Y - goal_pixel[1])**2)
        U_attr = self.K_ATTR * dist_goal

        dist_obst = distance_transform_edt(self.mapa_bin)
        U_rep = np.zeros_like(dist_obst, dtype=float)
        mask_near = (dist_obst > 0) & (dist_obst <= self.REP_RANGE)
        mask_on = (dist_obst == 0)
        d_norm = np.clip(dist_obst[mask_near] / self.REP_RANGE, 0, 1)
        U_rep[mask_near] = 0.5 * self.K_REP * (1 - d_norm)**2
        U_rep[mask_on] = np.inf

        U_total = U_attr + U_rep
        U_total_vis = np.copy(U_total)
        U_total_vis[mask_on] = 1e10

        Gy, Gx = np.gradient(U_total_vis)
        self.force_x = -Gx
        self.force_y = -Gy
        N = np.sqrt(self.force_x**2 + self.force_y**2)
        N[N == 0] = 1.0
        self.force_x /= N
        self.force_y /= N

        return U_total, X, Y

    def plot_potential_field(self, U_total, X, Y, goal_pixel):
        U_vis = np.copy(U_total)
        U_vis[U_vis == np.inf] = np.nan
        plt.figure(figsize=(12, 10))
        vmax = np.nanpercentile(U_vis, 99)
        plt.imshow(U_vis, cmap='viridis', origin='upper', vmax=vmax,
                   extent=(0, self.MAP_WIDTH_PIXELS, self.MAP_HEIGHT_PIXELS, 0))
        plt.colorbar(label="Potencial (U)")
        skip = (slice(None, None, 20), slice(None, None, 20))
        plt.quiver(X[skip], Y[skip], self.force_x[skip], self.force_y[skip],
                   color='white', pivot='middle', scale_units='xy', scale=0.1,
                   angles='xy')
        plt.plot(goal_pixel[0], goal_pixel[1], 'r*', markersize=15)
        plt.title("Campo Potencial")
        plt.show()

    def publish_debug_marker(self, publisher, x_world, y_world, r, g, b, ns, marker_id):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x_world)
        marker.pose.position.y = float(y_world)
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.lifetime = Duration(seconds=0).to_msg()
        publisher.publish(marker)

    def delete_debug_marker(self, publisher, ns, marker_id):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.ns = ns
        marker.id = marker_id
        marker.action = Marker.DELETE
        publisher.publish(marker)

    def odom_callback(self, msg: Odometry):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def send_velocity_command(self, linear_velocity, angular_velocity):
        twist_stamped_msg = TwistStamped()
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamped_msg.header.frame_id = 'base_link'
        twist_stamped_msg.twist.linear.x = linear_velocity
        twist_stamped_msg.twist.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(twist_stamped_msg)

    def is_free_pixel(self, x_px, y_px):
        if x_px < 0 or x_px >= self.MAP_WIDTH_PIXELS or y_px < 0 or y_px >= self.MAP_HEIGHT_PIXELS:
            return False
        return bool(self.mapa_bin[y_px, x_px])

    def find_free_point_around(self, robot_world, min_r=0.6, max_r=1.5, attempts=40):
        rx, ry = robot_world
        for _ in range(attempts):
            r = random.uniform(min_r, max_r)
            theta = random.uniform(-math.pi, math.pi)
            px = rx + r * math.cos(theta)
            py = ry + r * math.sin(theta)
            x_px, y_px = self.world_to_pixel(px, py)
            if self.is_free_pixel(x_px, y_px):
                return (px, py)
        return None

    def activate_temporary_goal(self, temp_goal_world):
        self.saved_real_goal = self.final_goal_world
        self.temporary_goal_active = True
        self.temporary_goal_world = temp_goal_world
        self.temporary_goal_start_time = self._now_seconds()
        self.recent_recovery_time = self._now_seconds()
        self.get_logger().info(f"Ativando goal temporário: {temp_goal_world}")
        self.publish_debug_marker(self.goal_marker_pub, temp_goal_world[0], temp_goal_world[1],
                                  r=0.0, g=0.0, b=1.0, ns="temp_goal", marker_id=1)
        self.process_new_goal(temp_goal_world)

    def restore_real_goal(self):
        if hasattr(self, "saved_real_goal") and self.saved_real_goal is not None:
            self.get_logger().info("Restaurando goal real.")
            self.delete_debug_marker(self.goal_marker_pub, "temp_goal", 1)
            self.temporary_goal_active = False
            self.temporary_goal_world = None
            self.temporary_goal_start_time = None
            self.process_new_goal(self.saved_real_goal)
            self.saved_real_goal = None

    def control_loop(self):
        if self.position is None or self.final_goal_world is None or self.force_x is None:
            return

        dx_goal = self.final_goal_world[0] - self.position[0]
        dy_goal = self.final_goal_world[1] - self.position[1]
        dist_to_goal = math.sqrt(dx_goal**2 + dy_goal**2)

        if dist_to_goal < self.GOAL_TOLERANCE:
            if self.temporary_goal_active:
                self.get_logger().info("Goal temporário alcançado, restaurando objetivo real.")
                self.restore_real_goal()
                return
            else:
                self.get_logger().info(f"Goal alcançado! Distância: {dist_to_goal:.3f} m")
                self.send_velocity_command(0.0, 0.0)
                self.final_goal_world = None
                self.force_x = None
                self.force_y = None
                self.delete_debug_marker(self.goal_marker_pub, "goal_marker", 0)
                return

        now_s = self._now_seconds()
        if self.last_dist_to_goal is None:
            self.last_dist_to_goal = dist_to_goal
            self.last_progress_time = now_s
        else:
            if self.last_dist_to_goal - dist_to_goal > self.PROGRESS_EPS:
                self.last_progress_time = now_s
                self.last_dist_to_goal = dist_to_goal
            else:
                time_since_progress = now_s - self.last_progress_time
                if (time_since_progress >= self.STUCK_TIME_THRESHOLD and
                    (now_s - self.recent_recovery_time) > self.MIN_TIME_BETWEEN_RECOVERIES):
                    self.get_logger().warn(f"Stuck detectado ({time_since_progress:.1f}s sem progresso). Tentando recovery...")
                    if not self.temporary_goal_active:
                        temp = self.find_free_point_around(self.position, min_r=1.5, max_r=3, attempts=200)
                        if temp is not None:
                            self.activate_temporary_goal(temp)
                        else:
                            self.get_logger().warn("Sem ponto livre próximo. Girando no lugar.")
                            self.send_velocity_command(0.0, 0.6)
                            self.recent_recovery_time = now_s
                            self.last_progress_time = now_s
                    else:
                        if now_s - self.temporary_goal_start_time > self.RECOVERY_DURATION:
                            self.get_logger().warn("Tempo de recovery expirado. Restaurando goal real.")
                            self.restore_real_goal()
                            self.last_progress_time = now_s
                            self.last_dist_to_goal = dist_to_goal

        robot_pixel_x, robot_pixel_y = self.world_to_pixel(self.position[0], self.position[1])
        fx_pixel = self.force_x[robot_pixel_y, robot_pixel_x]
        fy_pixel = self.force_y[robot_pixel_y, robot_pixel_x]
        fx_world = fx_pixel
        fy_world = -fy_pixel

        target_yaw = math.atan2(fy_world, fx_world)
        yaw_error = math.atan2(math.sin(target_yaw - self.yaw),
                               math.cos(target_yaw - self.yaw))

        linear = 0.0
        angular = 0.0

        if abs(yaw_error) > self.YAW_TOLERANCE:
            angular = self.ANGULAR_VEL_GAIN * yaw_error
        else:
            linear = self.LINEAR_VEL

        self.send_velocity_command(linear, angular)


def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Encerrando o nó e parando o robô.")
        node.send_velocity_command(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
