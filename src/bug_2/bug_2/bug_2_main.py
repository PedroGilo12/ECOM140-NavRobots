import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped # Importação corrigida
import math
import argparse

class Bug1Controller(Node):
    """
    Controlador de robô implementando o algoritmo Bug 1 para navegação em ambientes com obstáculos.
    """
    def __init__(self, goal_x, goal_y):
        super().__init__('bug1_controller')

        # Subs
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Pub
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10) # Publisher corrigido

        # Goal
        self.goal_x = goal_x
        self.goal_y = goal_y

        # Estados do algoritmo
        self.MODE_TO_GOAL = 0
        self.MODE_WALL_FOLLOWING = 1
        self.MODE_LEAVING_WALL = 2 # Novo estado para transição
        self.mode = self.MODE_TO_GOAL

        # Variáveis de estado
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.yaw = 0.0
        self.min_distance_to_wall = float('inf')

        # Variáveis específicas do Bug 1
        self.hit_point = None # Ponto onde o obstáculo foi encontrado
        self.closest_point_on_wall = None # Ponto no contorno mais próximo do objetivo
        self.min_distance_found = float('inf') # Distância do closest_point ao goal

        self.timer = self.create_timer(0.1, self.control_loop)

    def scan_callback(self, msg: LaserScan):
        """
        Callback do sensor LaserScan. Atualiza o estado de obstáculo detectado.
        """
        ranges = [r for r in msg.ranges if not math.isinf(r)]
        if ranges:
            self.min_distance_to_wall = min(ranges)
        else:
            self.min_distance_to_wall = float('inf')

    def odom_callback(self, msg: Odometry):
        """
        Callback do tópico de odometria. Atualiza a posição e orientação do robô.
        """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        

    def control_loop(self):
        """
        Loop de controle principal. Decide o movimento do robô baseado na posição do objetivo e obstáculos.
        """
        twist_stamped = TwistStamped() # Mensagem corrigida
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_link"
        
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        distance_to_goal = math.hypot(dx, dy)
        
        # Condição de parada: Chegou ao objetivo
        if distance_to_goal < 0.2:
            self.get_logger().info("Chegou no goal! ✅")
            self.cmd_pub.publish(twist_stamped)
            return

        if self.mode == self.MODE_TO_GOAL:
            self.move_to_goal(twist_stamped, distance_to_goal)
        elif self.mode == self.MODE_WALL_FOLLOWING:
            self.follow_wall(twist_stamped, distance_to_goal)
        elif self.mode == self.MODE_LEAVING_WALL:
            self.leave_wall(twist_stamped)
        
        self.cmd_pub.publish(twist_stamped)

    def move_to_goal(self, twist_stamped: TwistStamped, distance_to_goal: float):
        """Lógica para o modo 'Ir para o Objetivo'."""
        self.get_logger().info("Indo para o goal 🏃")
        if self.min_distance_to_wall < 0.4:
            self.get_logger().info("Obstáculo detectado. Iniciando contorno 🚧")
            self.mode = self.MODE_WALL_FOLLOWING
            self.hit_point = (self.robot_x, self.robot_y)
            self.closest_point_on_wall = (self.robot_x, self.robot_y)
            self.min_distance_found = distance_to_goal
            return

        target_theta = math.atan2(self.goal_y - self.robot_y, self.goal_x - self.robot_x)
        angle_diff = math.atan2(math.sin(target_theta - self.yaw), math.cos(target_theta - self.yaw))
        twist_stamped.twist.linear.x = 0.2
        twist_stamped.twist.angular.z = 2 * angle_diff

    def follow_wall(self, twist_stamped: TwistStamped, distance_to_goal: float):
        """Lógica para o modo 'Contornar Parede'."""
        self.get_logger().info("Contornando obstáculo...")

        # Atualiza o ponto mais próximo ao objetivo
        if distance_to_goal < self.min_distance_found:
            self.min_distance_found = distance_to_goal
            self.closest_point_on_wall = (self.robot_x, self.robot_y)
            self.get_logger().info(f"Novo ponto mais próximo encontrado: ({self.closest_point_on_wall[0]:.2f}, {self.closest_point_on_wall[1]:.2f})")

        # Verifica se completou o contorno (voltou ao hit_point)
        if math.hypot(self.robot_x - self.hit_point[0], self.robot_y - self.hit_point[1]) < 0.3: # Tolerância de 30cm
            if distance_to_goal > self.min_distance_found + 0.1: # Evita transição prematura se o hit_point for o ponto de saída ideal
                 self.get_logger().info("Contorno completo. Indo para o ponto de saída. 🚀")
                 self.mode = self.MODE_LEAVING_WALL
                 return

        # Lógica de contorno (ajustada para ser mais simples e funcional)
        if self.min_distance_to_wall < 0.3: # Muito perto da parede
            twist_stamped.twist.linear.x = 0.1
            twist_stamped.twist.angular.z = -0.5 # Gira para longe da parede
        elif self.min_distance_to_wall > 0.5: # Muito longe da parede
            twist_stamped.twist.linear.x = 0.1
            twist_stamped.twist.angular.z = 0.5 # Gira para perto da parede
        else: # Mantém distância ideal
            twist_stamped.twist.linear.x = 0.2
            twist_stamped.twist.angular.z = 0.0

    def leave_wall(self, twist_stamped: TwistStamped):
        """Lógica para o modo 'Sair do Contorno'."""
        self.get_logger().info("Indo para o ponto de saída... ➡️")
        dx = self.closest_point_on_wall[0] - self.robot_x
        dy = self.closest_point_on_wall[1] - self.robot_y
        distance_to_leave_point = math.hypot(dx, dy)

        if distance_to_leave_point < 0.2: # Chegou no ponto de saída
            self.get_logger().info("Chegou no ponto de saída. Retomando rota para o goal! ✅")
            self.mode = self.MODE_TO_GOAL
            return

        target_theta = math.atan2(dy, dx)
        angle_diff = math.atan2(math.sin(target_theta - self.yaw), math.cos(target_theta - self.yaw))
        twist_stamped.twist.linear.x = 0.2
        twist_stamped.twist.angular.z = 2 * angle_diff


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal_x", type=float, required=True, help="Coordenada X do objetivo.")
    parser.add_argument("--goal_y", type=float, required=True, help="Coordenada Y do objetivo.")
    parsed_args, _ = parser.parse_known_args()

    node = Bug1Controller(parsed_args.goal_x, parsed_args.goal_y)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()