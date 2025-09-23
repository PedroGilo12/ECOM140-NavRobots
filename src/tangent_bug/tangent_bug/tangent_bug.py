import rclpy
import math
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

import numpy as np

from enum import Enum

import argparse
from visualization_msgs.msg import MarkerArray

class State(Enum):
    MOVING_TO_GOAL = 1
    BONDARY_FOLLOWING = 2

class BoundaryStage(Enum):
    NONE = 0
    TURNING = 1
    FOLLOWING = 2

class TangentBug(Node):
    def __init__(self, goal_pos: list[float] = [0,0]):
        super().__init__('TangentBug')

        self.robot_radius = 0.3
        self.goal_x = goal_pos[0]
        self.goal_y = goal_pos[1]
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.laser_data = None
        self.tangent_array = []
        self.state = State.MOVING_TO_GOAL
        self.boundary_stage = BoundaryStage.NONE

        self.reach_pointer = []
        self.d_reach = float('inf')  
        self.followed_pointer = [] 
        self.d_followed = float('inf')   

        self.boundary_direction = 1
        self.boundary_distance = 0.6
        self.boundary_error_integral = 0.0
        self.kp_boundary = 1.6

        self.max_lidar_range = 1.5

        self.cmd_vel_publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.update_position,
            10
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.update_lidar,
            10
        )

        self.goal_marker_publisher = self.create_publisher(
            Marker,
            '/goal_marker',
            10
        )

        self.tangent_marker_publisher = self.create_publisher(
            Marker, 
            '/tangent_marker', 
            10)
        
        self.obstacle_marker_publisher = self.create_publisher(
            Marker, 
            '/obstacle_marker', 
            10)
        
        self.d_followed_publisher = self.create_publisher(
            Marker, 
            '/d_followed_marker', 
            10)
        
        self.state_publisher = self.create_publisher(
            String, 
            '/bug_state', 
            10)
        
        self.debug_markers = MarkerArray()
        self.marker_array_publisher = self.create_publisher(
            MarkerArray, 
            '/marker_array', 
            10)

        self.timer = self.create_timer(0.1, self.control_loop)

    def update_marker_array(self):
        self.marker_array_publisher.publish(self.debug_markers)

    def update_lidar(self, msg):
        filtered_ranges = [
            r if (math.isfinite(r) and r <= self.max_lidar_range) else math.inf
            for r in msg.ranges
        ]

        msg.ranges = filtered_ranges

        self.laser_data = msg
        self.tangent_array = self.find_discontinuities(1)

    def find_discontinuities(self, threshold=1):
        if self.laser_data is None:
            return []

        ranges = np.array(self.laser_data.ranges)
        discontinuities = []

        for i in range(len(ranges) - 1):
            r1 = ranges[i]
            r2 = ranges[i + 1]
            angle1 = self.laser_data.angle_min + i * self.laser_data.angle_increment
            angle2 = self.laser_data.angle_min + (i + 1) * self.laser_data.angle_increment

            if math.isfinite(r1):
                x = (r1) * math.cos(angle1)
                y = (r1) * math.sin(angle1)
                x_world = self.current_x + (x * math.cos(self.current_yaw) - y * math.sin(self.current_yaw))
                y_world = self.current_y + (x * math.sin(self.current_yaw) + y * math.cos(self.current_yaw))
                distance_to_goal = math.hypot(x_world - self.goal_x, y_world - self.goal_y)
                if (distance_to_goal < self.d_followed) and (self.state == State.MOVING_TO_GOAL):
                    self.d_followed = distance_to_goal
                    self.followed_pointer = [x_world,y_world]

            if math.isfinite(r1) and not math.isfinite(r2):
                x = (r1) * math.cos(angle1)
                y = (r1) * math.sin(angle1)
                discontinuities.append([x, y])
                continue

            if not math.isfinite(r1) and math.isfinite(r2):
                x = (r2) * math.cos(angle2)
                y = (r2) * math.sin(angle2)
                discontinuities.append([x, y])
                continue

            if math.isfinite(r1) and math.isfinite(r2):
                if abs(r1 - r2) > threshold:
                    x = (r1) * math.cos(angle1)
                    y = (r1) * math.sin(angle1)
                    discontinuities.append([x, y])
                    continue

        return discontinuities
    
    def update_position(self, msg):
        """
        Callback para atualizar a posição e orientação (yaw) do robô.
        """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        
        q_x = orientation_q.x
        q_y = orientation_q.y
        q_z = orientation_q.z
        q_w = orientation_q.w
        
        t3 = +2.0 * (q_w * q_z + q_x * q_y)
        t4 = +1.0 - 2.0 * (q_y * q_y + q_z * q_z)
        self.current_yaw = math.atan2(t3, t4)
    
    def send_velocity_command(self, linear_velocity, angular_velocity):
        """
        Cria e publica uma mensagem TwistStamped no tópico /cmd_vel.
        """
        twist_stamped_msg = TwistStamped()

        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamped_msg.header.frame_id = 'base_link' 

        twist_stamped_msg.twist.linear.x = linear_velocity
        twist_stamped_msg.twist.angular.z = angular_velocity
        
        self.cmd_vel_publisher.publish(twist_stamped_msg)

    def control_loop(self):
        self.publish_goal_marker()

        if math.isfinite(self.d_followed):
            self.publish_debug_marker(
                publisher=self.d_followed_publisher,
                x_world=self.followed_pointer[0],
                y_world=self.followed_pointer[1],
                r=1.0, g=0.0, b=0.0,
                ns="d_followed",
                marker_id=3
            )

        msg = String()
        if self.state == State.MOVING_TO_GOAL:
            msg.data = "Motion to Goal"
            self.state_publisher.publish(msg)
            self.move_to_goal()
        elif self.state == State.BONDARY_FOLLOWING:
            msg.data = "Boundary Following"
            self.state_publisher.publish(msg)
            self.boundary_follow()

    def get_global_coordinates(self, r, angle_lidar):
        x_local = r * math.cos(angle_lidar)
        y_local = r * math.sin(angle_lidar)
        x_world = self.current_x + (x_local * math.cos(self.current_yaw) - y_local * math.sin(self.current_yaw))
        y_world = self.current_y + (x_local * math.sin(self.current_yaw) + y_local * math.cos(self.current_yaw))
        return x_world, y_world

    def obstacle_in_range(self, angle_start, angle_end, distance_threshold, color=(0.0, 1.0, 0.0), id=4):
        total_count = len(self.laser_data.ranges)

        idx_start = int((angle_start - self.laser_data.angle_min) / self.laser_data.angle_increment) % total_count
        idx_end = int((angle_end - self.laser_data.angle_min) / self.laser_data.angle_increment) % total_count

        start_detected = 0.0
        end_detected = 0.0

        if idx_start <= idx_end:
            indices = list(range(idx_start, idx_end + 1))
        else:
            indices = list(range(idx_start, total_count)) + list(range(0, idx_end + 1))

        r_start = self.laser_data.ranges[idx_start]
        r_end = self.laser_data.ranges[idx_end]

        if math.isfinite(r_start) and r_start < distance_threshold:
            x_world, y_world = self.get_global_coordinates(r_start, angle_start)
            start_detected = 1.0
        else:
            x_world, y_world = self.get_global_coordinates(distance_threshold, angle_start)
        
        if math.isfinite(r_end) and r_end < distance_threshold:
            x_world_end, y_world_end = self.get_global_coordinates(r_end, angle_end)
            end_detected = 1.0
        else:
            x_world_end, y_world_end = self.get_global_coordinates(distance_threshold, angle_end)

        start_marker = self.create_debug_marker(
            publisher=self.obstacle_marker_publisher,
            x_world=x_world,
            y_world=y_world,
            r=start_detected, g=0, b=1,
            ns="obstacle_start",
            marker_id=id
        )

        end_marker = self.create_debug_marker(
            publisher=self.obstacle_marker_publisher,
            x_world=x_world_end,
            y_world=y_world_end,
            r=end_detected, g=1, b=0,
            ns="obstacle_end",
            marker_id=id+1
        )

        self.debug_markers.markers.append(start_marker)
        self.debug_markers.markers.append(end_marker)

        self.update_marker_array()

        min_range = distance_threshold
        for i in indices:
            if math.isfinite(self.laser_data.ranges[i]) and self.laser_data.ranges[i] < min_range:
                min_range = self.laser_data.ranges[i]
        return min_range

    def boundary_follow(self):
        if self.laser_data is None or not self.tangent_array:
            return
        self.get_logger().info(f"boundary follow")

        if self.boundary_stage == BoundaryStage.NONE:
            self.reach_pointer = self.return_best_discontinuitie()
            self.boundary_stage = BoundaryStage.TURNING

        x, y = self.return_best_discontinuitie()
        distance_to_reach = math.hypot(x - self.goal_x, y - self.goal_y)

        obstacle_in_front = self.obstacle_in_range(-math.pi/6, math.pi/6, 0.5, id=4)
        obstacle_in_right = self.obstacle_in_range(self.boundary_direction * math.pi/2, self.boundary_direction * math.pi/6, 0.8, id=6)

        if distance_to_reach < self.d_followed:
            self.reach_pointer = []
            self.d_reach = float('inf')  
            self.followed_pointer = [] 
            self.d_followed = float('inf')
            self.state = State.MOVING_TO_GOAL
            self.boundary_stage = BoundaryStage.NONE
            self.get_logger().info("Switching to MOVING_TO_GOAL")
            return   

        if self.boundary_stage == BoundaryStage.TURNING:
            x_obs, y_obs = self.reach_pointer 
            distance_to_reach = math.hypot(x_obs - self.current_x, y_obs - self.current_y)
            target_angle = math.atan2(y_obs - self.current_y, x_obs - self.current_x)
            angle_error = target_angle - self.current_yaw
            if angle_error > math.pi:
                angle_error -= 2 * math.pi
            elif angle_error < -math.pi:
                angle_error += 2 * math.pi

            if abs(angle_error) > 0.1:
                angular_velocity = 0.8 * angle_error
                self.send_velocity_command(0.0, angular_velocity)
            elif distance_to_reach > 0.45:
                self.send_velocity_command(0.20, 0.0)
            else:

                [x_r, y_r] = self.get_closest_obstacle()
                angle_reach = math.atan2(y_r - self.current_y, x_r - self.current_x)
                relative_angle = angle_reach - self.current_yaw

                if relative_angle > math.pi:
                    relative_angle -= 2 * math.pi
                elif relative_angle < -math.pi:
                    relative_angle += 2 * math.pi

                if (relative_angle < 0):
                    self.boundary_direction = -1
                else:
                    self.boundary_direction = 1

                self.get_logger().info(f"Reach pointer relative angle: {relative_angle:.2f} rad")

                self.boundary_stage = BoundaryStage.FOLLOWING
                return
        elif self.boundary_stage == BoundaryStage.FOLLOWING:
            self.get_logger().info(f"Following boundary")
            if self.laser_data is None:
                return

            if obstacle_in_front < 0.45:
                self.get_logger().info("Obstacle detected in front during boundary following.")
                self.send_velocity_command(0.0, self.boundary_direction * -0.5)
            else:
                # boundary_error = obstacle_in_right - self.boundary_distance
                # angular_velocity = self.boundary_direction * self.kp_boundary * (boundary_error)

                angular_velocity = 0.0
                if obstacle_in_right >= 0.3:
                    self.get_logger().info("Right obstacle far, turning right")
                    angular_velocity = 1.0

                self.get_logger().info("Right obstacle distance: {:.2f}, angular velocity: {:.2f}".format(obstacle_in_right, angular_velocity))
                self.send_velocity_command(0.20, self.boundary_direction * angular_velocity)

    def get_closest_obstacle(self):
        if self.laser_data is None:
            return [0.0,0.0] 

        min_dist_laser = float('inf')
        min_index_laser = -1

        for i, dist in enumerate(self.laser_data.ranges):
            if math.isfinite(dist) and dist > 0 and dist < min_dist_laser:
                min_dist_laser = dist
                min_index_laser = i

        if min_index_laser == -1:
            return [0.0,0.0]

        angle_obstacle = self.laser_data.angle_min + min_index_laser * self.laser_data.angle_increment

        x_local = min_dist_laser * math.cos(angle_obstacle)
        y_local = min_dist_laser * math.sin(angle_obstacle)
        x_world = self.current_x + (x_local * math.cos(self.current_yaw) - y_local * math.sin(self.current_yaw))
        y_world = self.current_y + (x_local * math.sin(self.current_yaw) + y_local * math.cos(self.current_yaw))

        self.publish_debug_marker(
                publisher=self.obstacle_marker_publisher,
                x_world=x_world,
                y_world=y_world,
                r=0.0, g=0.0, b=1.0,
                ns="closest_obstacle",
                marker_id=2
            )

        return [x_world, y_world]

    def deviate_from_closest_obstacle(self):
        """
        Função de emergência para quando o robô fica preso perto de um local_goal.
        Encontra o obstáculo mais próximo e retorna uma velocidade angular para girar para longe dele.
        """
        if self.laser_data is None:
            return 0.0  # Não há nada a desviar

        min_dist_laser = float('inf')
        min_index_laser = -1

        for i, dist in enumerate(self.laser_data.ranges):
            if math.isfinite(dist) and dist > 0 and dist < min_dist_laser:
                min_dist_laser = dist
                min_index_laser = i

        if min_index_laser == -1:
            return 0.0

        angle_obstacle = self.laser_data.angle_min + min_index_laser * self.laser_data.angle_increment

        x_local = min_dist_laser * math.cos(angle_obstacle)
        y_local = min_dist_laser * math.sin(angle_obstacle)
        x_world = self.current_x + (x_local * math.cos(self.current_yaw) - y_local * math.sin(self.current_yaw))
        y_world = self.current_y + (x_local * math.sin(self.current_yaw) + y_local * math.cos(self.current_yaw))

        avoidance_gain = 0.8
        self.get_logger().info(f"Angle obstacle: {angle_obstacle}")
        if min_dist_laser < 0.4:
            self.avoidance =True

            self.publish_debug_marker(
                publisher=self.obstacle_marker_publisher,
                x_world=x_world,
                y_world=y_world,
                r=0.0, g=0.0, b=1.0,
                ns="closest_obstacle",
                marker_id=2
            )
            if angle_obstacle >= math.pi:
                return avoidance_gain / (min_dist_laser + 1e-3)  
            else:
                return -avoidance_gain / (min_dist_laser + 1e-3) 
        else:
            return 0
    
    def move_to_goal(self):
        self.get_logger().info(f"Move to goal")
        avoidance_angle = 0.0

        if self.laser_data == None:
            return
        
        angle_to_goal_world = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        angle_error = angle_to_goal_world - self.current_yaw
        
        dist_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)

        if self.movement_is_blocked(angle_error):
            current_goal = self.return_best_discontinuitie()
            current_goal_x = current_goal[0]
            current_goal_y = current_goal[1]

            distance_to_current_goal = math.hypot(self.goal_x - current_goal_x, self.goal_y - current_goal_y)

            if distance_to_current_goal > dist_to_goal:
                self.state = State.BONDARY_FOLLOWING
                return

            angle_to_goal_world = math.atan2(current_goal_y - self.current_y, current_goal_x - self.current_x)
        else:
            angle_to_goal_world = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        
        angle_error = angle_to_goal_world - self.current_yaw
        
        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        avoidance_angle = self.deviate_from_closest_obstacle()
        angular_vel = (0.75 * angle_error) + avoidance_angle
        linear_vel = 0.20

        if dist_to_goal < 0.2:
            self.send_velocity_command(0.0, 0.0)
            self.timer.cancel()
        else:
            self.send_velocity_command(linear_vel, angular_vel)

    def return_best_discontinuitie(self):
        if self.laser_data is None or not self.tangent_array:
            return

        angle_to_goal_world = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        angle_error = angle_to_goal_world - self.current_yaw

        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2 * math.pi

        best_tangent = None
        min_dist = float('inf')

        for tangent in self.tangent_array:
            x_local, y_local = tangent

            x_world = self.current_x + (x_local * math.cos(self.current_yaw) - y_local * math.sin(self.current_yaw))
            y_world = self.current_y + (x_local * math.sin(self.current_yaw) + y_local * math.cos(self.current_yaw))

            dist = math.hypot(self.goal_x - x_world, self.goal_y - y_world) + math.hypot(self.current_x - x_world, self.current_y - y_world)
            if dist < min_dist:
                min_dist = dist
                best_tangent = [x_world, y_world]

        local_goal_x, local_goal_y = best_tangent

        if self.state == State.MOVING_TO_GOAL:
            self.publish_debug_marker(
                publisher=self.tangent_marker_publisher,
                x_world=local_goal_x,
                y_world=local_goal_y,
                r=0.0, g=1.0, b=0.0,
                ns="tangent_point",
                marker_id=1
            )
        else:
            self.publish_debug_marker(
                publisher=self.tangent_marker_publisher,
                x_world=local_goal_x,
                y_world=local_goal_y,
                r=1.0, g=1.0, b=0.0, 
                ns="tangent_point",
                marker_id=1
            )

        return best_tangent
            
    def movement_is_blocked(self, angle_error):
        index_do_goal = int(angle_error / self.laser_data.angle_increment)
        total_ranges = len(self.laser_data.ranges)
        index_do_goal = (index_do_goal + total_ranges) % total_ranges

        obstacle_distance = self.laser_data.ranges[index_do_goal]
        distance_to_goal = math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)

        if math.isfinite(obstacle_distance) and (distance_to_goal > obstacle_distance):
            return True
                
        return False

    def publish_goal_marker(self):
        """
        Cria e publica um Marker no RViz para visualizar a posição do goal.
        """
        marker = Marker()
        
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal_namespace"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        try:
            marker.pose.position.x = float(self.goal_x)
            marker.pose.position.y = float(self.goal_y)
            marker.pose.position.z = 0.0
        except (TypeError, ValueError) as e:
            return

        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.lifetime.sec = 5

        # Publica o marcador
        self.goal_marker_publisher.publish(marker)
    
    def create_debug_marker(self, publisher, x_world, y_world, r, g, b, ns, marker_id):
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
        
        marker.scale.x = 0.07 
        marker.scale.y = 0.07
        marker.scale.z = 0.07
        
        marker.color.a = 1.0
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        
        marker.lifetime.sec = 5
        
        return marker
    
    def publish_debug_marker(self, publisher, x_world, y_world, r, g, b, ns, marker_id):
        """Publica um pequeno marcador esférico para depuração."""
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
        
        marker.scale.x = 0.15 
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        
        marker.color.a = 1.0
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        
        marker.lifetime.sec = 5
        
        publisher.publish(marker)

def main(args=None):
    parser = argparse.ArgumentParser(description='TangentBug Node')
    parser.add_argument('--goal', type=float, nargs=2, default=[-2, 3], help='Goal coordinates as x y')
    args, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)

    print(args.goal)

    minimal_subscriber = TangentBug(args.goal)

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()