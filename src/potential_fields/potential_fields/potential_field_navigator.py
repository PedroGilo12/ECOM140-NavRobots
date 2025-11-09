import rclpy
from rclpy.node import Node
from rclpy.duration import Duration # <--- ADICIONADO
from geometry_msgs.msg import Point, TwistStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker # <--- ADICIONADO
import math
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import matplotlib.pyplot as plt

class PotentialFieldNavigator(Node):
    def __init__(self):
        super().__init__('potential_field_navigator')

        # --- Parâmetros (Ajustáveis) ---
        
        self.INITIAL_GOAL_WORLD = (2.5, -2.0) 
        self.MAP_IMAGE_PATH = 'mapa_rrt.png' 
        
        self.K_ATTR = 0.5                   
        self.K_REP = 100.0                  
        self.REP_RANGE = 100                
        
        self.CONTROL_LOOP_PERIOD = 0.1      
        self.GOAL_TOLERANCE = 0.15          
        self.YAW_TOLERANCE = 0.1            
        self.LINEAR_VEL = 0.25              
        self.ANGULAR_VEL_GAIN = 1.8         
        
        # --- Publishers e Subscribers ---
        self.goal_sub = self.create_subscription(Point, 'goal', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # --- ADICIONADO: Publisher do Marcador ---
        self.goal_marker_pub = self.create_publisher(Marker, '/goal_marker', 10)
        # (O '/tangent_marker' do RRT não é usado aqui, pois só temos um goal)
        
        # --- Estado do Robô ---
        self.position = None  
        self.yaw = 0.0        

        # --- Estado do Planejamento ---
        self.final_goal_world = None 
        self.force_x = None          
        self.force_y = None          
        self.mapa_bin = None         
        self.is_first_plot = True    

        # --- Configuração do Mapa e Transformação ---
        try:
            self.load_map(self.MAP_IMAGE_PATH)
            self.get_logger().info(f"Mapa '{self.MAP_IMAGE_PATH}' carregado com sucesso.")
            self.get_logger().info(f"Dimensões do Mapa (Pixels): {self.MAP_WIDTH_PIXELS}x{self.MAP_HEIGHT_PIXELS}")
        except FileNotFoundError:
            self.get_logger().error(f"Erro: Imagem do mapa não encontrada em '{self.MAP_IMAGE_PATH}'")
            rclpy.shutdown()
            return

        # Escalas para conversão
        self.MAP_WIDTH_METERS = 8.0
        self.MAP_HEIGHT_METERS = 8.0
        self.scale_x = self.MAP_WIDTH_METERS / self.MAP_WIDTH_PIXELS
        self.scale_y = self.MAP_HEIGHT_METERS / self.MAP_HEIGHT_PIXELS

        # --- Processa o Goal Hardcoded Inicial ---
        self.get_logger().info(f"Processando goal inicial hardcoded: {self.INITIAL_GOAL_WORLD}")
        self.process_new_goal(self.INITIAL_GOAL_WORLD)

        # --- Iniciar Loop de Controle ---
        self.timer = self.create_timer(self.CONTROL_LOOP_PERIOD, self.control_loop)
        self.get_logger().info("Nó pronto. Iniciando loop de controle do robô.")

    def load_map(self, image_path):
        """Carrega a imagem do mapa e a converte em um array numpy binário."""
        img = Image.open(image_path).convert('L')
        img_np = np.array(img)
        self.mapa_bin = (img_np > 127).astype(np.uint8)
        self.MAP_HEIGHT_PIXELS, self.MAP_WIDTH_PIXELS = self.mapa_bin.shape

    def pixel_to_world(self, x_pixel, y_pixel):
        """Converte coordenadas de pixel (imagem) para coordenadas ROS (odom)."""
        x_world = (x_pixel - self.MAP_WIDTH_PIXELS / 2) * self.scale_x
        y_world = (self.MAP_HEIGHT_PIXELS / 2 - y_pixel) * self.scale_y
        return x_world, y_world

    def world_to_pixel(self, x_world, y_world):
        """Converte coordenadas ROS (odom) para coordenadas de pixel (imagem)."""
        x_pixel = (x_world / self.scale_x) + (self.MAP_WIDTH_PIXELS / 2)
        y_pixel = (self.MAP_HEIGHT_PIXELS / 2) - (y_world / self.scale_y)
        
        x_pixel = int(np.clip(x_pixel, 0, self.MAP_WIDTH_PIXELS - 1))
        y_pixel = int(np.clip(y_pixel, 0, self.MAP_HEIGHT_PIXELS - 1))
        return x_pixel, y_pixel

    def goal_callback(self, msg: Point):
        """Recebe um novo goal, o armazena e recalcula o campo potencial."""
        self.process_new_goal((msg.x, msg.y))

    def process_new_goal(self, goal_world_coords):
        """Processa um novo goal, recalcula o campo e plota (se for a primeira vez)."""
        self.final_goal_world = goal_world_coords
        self.get_logger().info(f"Novo goal mundial recebido: x={self.final_goal_world[0]:.2f}, y={self.final_goal_world[1]:.2f}")

        goal_pixel_x, goal_pixel_y = self.world_to_pixel(self.final_goal_world[0], self.final_goal_world[1])
        
        if self.mapa_bin[goal_pixel_y, goal_pixel_x] == 0:
            self.get_logger().warn("O 'goal' está dentro de um obstáculo! Navegação cancelada.")
            self.final_goal_world = None
            self.force_x = None
            self.force_y = None
            # Remove o marcador se o goal for inválido
            self.delete_debug_marker(self.goal_marker_pub, "goal_marker", 0)
            return

        self.get_logger().info(f"Goal em pixels: x={goal_pixel_x}, y={goal_pixel_y}. Calculando campo potencial...")
        
        U_total, X, Y = self.calculate_potential_field((goal_pixel_x, goal_pixel_y))
        
        self.get_logger().info("Campo potencial e gradiente calculados.")
        
        # --- ADICIONADO: Publica o marcador do goal no RViz ---
        self.publish_debug_marker(
            self.goal_marker_pub,
            self.final_goal_world[0], # x_world
            self.final_goal_world[1], # y_world
            r=1.0, g=0.0, b=0.0,       # Cor: Vermelho
            ns="goal_marker",
            marker_id=0
        )
        # --- FIM DA ADIÇÃO ---

        if self.is_first_plot:
            self.get_logger().info("Exibindo gráfico do campo potencial...")
            self.get_logger().warn("FECHE A JANELA DO GRÁFICO PARA INICIAR A NAVEGAÇÃO DO ROBÔ.")
            self.plot_potential_field(U_total, X, Y, (goal_pixel_x, goal_pixel_y))
            self.is_first_plot = False 
            self.get_logger().info("Gráfico fechado. Iniciando navegação.")

    def calculate_potential_field(self, goal_pixel):
        """
        Gera o campo potencial e armazena o gradiente (força) normalizado.
        (Lógica do seu script)
        """
        h, w = self.mapa_bin.shape
        Y, X = np.mgrid[0:h, 0:w] 

        # Campo Atrativo
        dist_goal = np.sqrt((X - goal_pixel[0])**2 + (Y - goal_pixel[1])**2)
        U_attr = self.K_ATTR * dist_goal

        # Campo Repulsivo
        dist_obst = distance_transform_edt(self.mapa_bin) 
        U_rep = np.zeros_like(dist_obst, dtype=float)
        
        mask_near = (dist_obst > 0) & (dist_obst <= self.REP_RANGE)
        mask_on = (dist_obst == 0)

        d_norm = np.clip(dist_obst[mask_near] / self.REP_RANGE, 0, 1)
        U_rep[mask_near] = 0.5 * self.K_REP * (1 - d_norm)**2
        U_rep[mask_on] = np.inf
        
        # Campo Total
        U_total = U_attr + U_rep
        U_total_vis = np.copy(U_total)
        U_total_vis[mask_on] = 1e10 

        # Gradiente (Força) - (Sinal do seu script: -Gx, +Gy)
        Gy, Gx = np.gradient(U_total_vis)
        self.force_x = -Gx 
        self.force_y = -Gy  

        # Normalizar
        N = np.sqrt(self.force_x**2 + self.force_y**2)
        N[N == 0] = 1.0 
        
        self.force_x = self.force_x / N
        self.force_y = self.force_y / N

        return U_total, X, Y

    def plot_potential_field(self, U_total, X, Y, goal_pixel):
        """
        Usa Matplotlib para exibir o campo potencial (heatmap) e
        o campo de força (vetores).
        """
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

        plt.plot(goal_pixel[0], goal_pixel[1], 'r*', markersize=15, label=f"Goal (Pixel: {goal_pixel})")
        
        plt.title("Campo Potencial e Campo de Força (Gradiente)")
        plt.xlabel("Coordenada X (pixels)")
        plt.ylabel("Coordenada Y (pixels)")
        plt.legend()
        plt.axis('equal')
        plt.show() # BLOQUEANTE

    # --- FUNÇÃO HELPER DE MARCADOR ADICIONADA ---
    def publish_debug_marker(self, publisher, x_world, y_world, r, g, b, ns, marker_id):
        """Publica um pequeno marcador esférico para depuração."""
        marker = Marker()
        marker.header.frame_id = "odom" # Publica no frame 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = float(x_world)
        marker.pose.position.y = float(y_world)
        marker.pose.position.z = 0.1  # Um pouco acima do chão
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.2 # 20cm
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        marker.color.a = 1.0 # Opacidade total
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        
        # Lifetime 0 = persiste para sempre (ou até ser atualizado/deletado)
        marker.lifetime = Duration(seconds=0).to_msg()
        
        publisher.publish(marker)

    # --- FUNÇÃO HELPER DE MARCADOR ADICIONADA ---
    def delete_debug_marker(self, publisher, ns, marker_id):
        """Publica um marcador com ação de DELETE."""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.ns = ns
        marker.id = marker_id
        marker.action = Marker.DELETE
        publisher.publish(marker)

    def odom_callback(self, msg: Odometry):
        """Atualiza a posição e orientação (yaw) do robô."""
        self.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def send_velocity_command(self, linear_velocity, angular_velocity):
        """Publica uma mensagem TwistStamped no tópico /cmd_vel."""
        twist_stamped_msg = TwistStamped()
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamped_msg.header.frame_id = 'base_link' 
        twist_stamped_msg.twist.linear.x = linear_velocity
        twist_stamped_msg.twist.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(twist_stamped_msg)
    
    def control_loop(self):
        """Loop principal de controle que calcula e envia comandos de velocidade."""
        
        if self.position is None or self.final_goal_world is None or self.force_x is None:
            if self.is_first_plot: 
                pass
            else:
                self.get_logger().info("Aguardando Posição e Goal...", throttle_duration_sec=5.0)
            return

        # --- 1. Verificar se o Goal foi alcançado ---
        dx_goal = self.final_goal_world[0] - self.position[0]
        dy_goal = self.final_goal_world[1] - self.position[1] # <--- CORREÇÃO: era self.position[0]
        dist_to_goal = math.sqrt(dx_goal**2 + dy_goal**2)

        if dist_to_goal < self.GOAL_TOLERANCE:
            self.get_logger().info(f"Goal alcançado! (Distância: {dist_to_goal:.3f}m)")
            self.send_velocity_command(0.0, 0.0)
            self.final_goal_world = None 
            self.force_x = None          
            self.force_y = None
            
            # --- ADICIONADO: Remove o marcador do goal ---
            self.delete_debug_marker(self.goal_marker_pub, "goal_marker", 0)
            # --- FIM DA ADIÇÃO ---
            return

        # --- 2. Obter Vetor de Força na Posição Atual ---
        robot_pixel_x, robot_pixel_y = self.world_to_pixel(self.position[0], self.position[1])

        fx_pixel = self.force_x[robot_pixel_y, robot_pixel_x]
        fy_pixel = self.force_y[robot_pixel_y, robot_pixel_x]

        # --- 3. Converter Vetor de Força (Pixel) para Ângulo (Mundo) ---
        fx_world = fx_pixel
        fy_world = -fy_pixel # Inverte o eixo Y

        target_yaw = math.atan2(fy_world, fx_world)

        # --- 4. Calcular Erro e Comando (Controlador P) ---
        yaw_error = math.atan2(math.sin(target_yaw - self.yaw),
                               math.cos(target_yaw - self.yaw))

        linear = 0.0
        angular = 0.0

        if abs(yaw_error) > self.YAW_TOLERANCE:
            angular = self.ANGULAR_VEL_GAIN * yaw_error
        else:
            linear = self.LINEAR_VEL

        # --- 5. Enviar Comando ---
        self.send_velocity_command(linear, angular)


def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Desligando o nó, parando o robô.")
        node.send_velocity_command(0.0, 0.0) # Para o robô
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()