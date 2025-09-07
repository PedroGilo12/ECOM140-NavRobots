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

class State(Enum):
    MOVING_TO_GOAL = 1
    BONDARY_FOLLOWING = 2

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

        self.d_min_global = float('inf')   
        self.boundary_start = None         
        self.d_followed = 0.0              
        self.last_position = None

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
            '/goal_marker', # Nome do tópico para o marcador
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

        self.timer = self.create_timer(0.1, self.control_loop)

    def update_lidar(self, msg):
        self.laser_data = msg
        self.tangent_array= self.find_discontinuities(1.5)

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
        if self.state == State.MOVING_TO_GOAL:
            self.move_to_goal()
        elif self.state == State.BONDARY_FOLLOWING:
            self.bondary_follow()

    def bondary_follow(self):
        if self.laser_data is None or not self.tangent_array:
            return

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

        avoidance_gain = 0.75
        self.get_logger().info(f"Angle obstacle: {angle_obstacle}")
        if min_dist_laser < 0.4:
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
        
        if self.movement_is_blocked(angle_error):
            avoidance_angle = self.deviate_from_closest_obstacle()
            current_goal = self.return_best_discontinuitie()
            current_goal_x = current_goal[0]
            current_goal_y = current_goal[1]
            angle_to_goal_world = math.atan2(current_goal_y - self.current_y, current_goal_x - self.current_x)
        else:
            angle_to_goal_world = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        
        angle_error = angle_to_goal_world - self.current_yaw
        
        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2 * math.pi
                
        angular_vel = (0.5 * angle_error) + avoidance_angle
        dist_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
        linear_vel = 0.26

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

        self.publish_debug_marker(
            publisher=self.tangent_marker_publisher,
            x_world=local_goal_x,
            y_world=local_goal_y,
            r=0.0, g=1.0, b=0.0,  # Verde
            ns="tangent_point",
            marker_id=1
        )

        return best_tangent
            
    def movement_is_blocked(self, angle_error):

        index_do_goal = int(angle_error / self.laser_data.angle_increment)
        
        total_ranges = len(self.laser_data.ranges)
        index_do_goal = (index_do_goal + total_ranges) % total_ranges

        distancia_na_direcao_do_goal = self.laser_data.ranges[index_do_goal]

        if math.isfinite(distancia_na_direcao_do_goal):
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
        
        marker.lifetime.sec = 1
        
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