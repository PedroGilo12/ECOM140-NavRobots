import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, TwistStamped
from nav_msgs.msg import Odometry
import math
import rrt_planning.rrt_module
from rrt_planning.rrt_module import run_rrt
from visualization_msgs.msg import Marker

class MotionToGoal(Node):
    def __init__(self):
        super().__init__('motion_to_goal')

        self.goal_sub = self.create_subscription(Point, 'goal', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(Point, '/goal', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/goal_marker', 10)
        self.end_goal_marker_pub = self.create_publisher(Marker, '/tangent_marker', 10)

        self.goal = None
        self.goal_updated = False
        self.position = None
        self.yaw = 0.0

        self.MAP_WIDTH_PIXELS = 800   # ajuste conforme seu mapa
        self.MAP_HEIGHT_PIXELS = 800
        self.MAP_WIDTH_METERS = 8
        self.MAP_HEIGHT_METERS = 8
        self.scale_x = self.MAP_WIDTH_METERS / self.MAP_WIDTH_PIXELS
        self.scale_y = self.MAP_HEIGHT_METERS / self.MAP_HEIGHT_PIXELS

        self.path = run_rrt(
            step=15,
            image_path='mapa_rrt.png',
            robot_size=(40, 40),
            start=(400, 400, 0),
            goal=(10, 10, 0),
            gui=True,
            obstacle_bias=False,
        )

        while True:
            pass

        self.end_goal = (self.path[-1].x, self.path[-1].y)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.goal_timer = self.create_timer(0.1, self.update_goal)

    def pixel_to_world(self, x_pixel, y_pixel):
        """
        Converte coordenadas de pixel (imagem) para coordenadas ROS (odom).
        Corrige a origem e inverte o eixo Y.
        """
        x_world = (x_pixel - self.MAP_WIDTH_PIXELS / 2) * self.scale_x
        y_world = (self.MAP_HEIGHT_PIXELS / 2 - y_pixel) * self.scale_y
        return x_world, y_world

    def update_goal(self):
        if not self.goal_updated:
            self.goal_updated = True
            current_goal: rrt_planning.rrt_module.Pose = self.path[0]

            self.path.remove(current_goal)
            x_world, y_world = self.pixel_to_world(current_goal.x, current_goal.y)

            goal_point = Point()
            goal_point.x = x_world
            goal_point.y = y_world
            goal_point.z = 0.0
            self.goal_pub.publish(goal_point)

            self.goal = (x_world, y_world)

            self.publish_debug_marker(
                self.goal_marker_pub,
                x_world,
                y_world,
                r=1.0, g=0.0, b=0.0,
                ns="goal_marker",
                marker_id=0
            )
             
    def goal_callback(self, msg: Point):
        self.goal = (msg.x, msg.y)
        self.get_logger().info(f"Novo goal recebido: x={msg.x:.2f}, y={msg.y:.2f}")

    def odom_callback(self, msg: Odometry):
        self.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

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

    def control_loop(self):
        if self.goal is None or self.position is None:
            self.get_logger().info(f"Error")
            return

        self.publish_debug_marker(
            self.goal_marker_pub,
            self.goal[0],
            self.goal[1],
            r=1.0, g=0.0, b=0.0,
            ns="goal_marker",
            marker_id=0
        )

        x_end, y_end = self.pixel_to_world(self.end_goal[0], self.end_goal[1])

        self.publish_debug_marker(
            self.end_goal_marker_pub,
            x_end,
            y_end,
            r=0.0, g=1.0, b=0.0,  # cor diferente para distinguir
            ns="tangent_marker",
            marker_id=0
        )

        dx = self.goal[0] - self.position[0]
        dy = self.goal[1] - self.position[1]
        target_yaw = math.atan2(dy, dx)

        yaw_error = math.atan2(math.sin(target_yaw - self.yaw),
                               math.cos(target_yaw - self.yaw))

        dist_error = math.sqrt(dx*dx + dy*dy)

        linear = 0.0
        angular = 0.0

        if abs(yaw_error) > 0.05:
            angular = 3 * yaw_error
        elif dist_error > 0.05:
            linear = 0.2
        else:
            self.get_logger().info("Goal alcançado!")
            self.goal_updated = False
            self.goal = None

        self.send_velocity_command(linear, angular)


def main(args=None):
    rclpy.init(args=args)
    node = MotionToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
