import pygame
import numpy as np
from PIL import Image, ImageDraw
from scipy.ndimage import binary_dilation, rotate, distance_transform_edt
import time
from PIL import ImageOps
import math
import random
from typing import Callable, List, Optional, Tuple
import os

def toRGBArray(matriz):
    imagem = (1 - matriz) * 255
    rgb_array = np.stack([imagem] * 3, axis=-1)
    return rgb_array

def toRGBArrayColored(matriz, cor=(255, 0, 0)):
    fundo_transparente = (255, 0, 255) 
    altura, largura = matriz.shape
    
    rgb_array = np.full((altura, largura, 3), fundo_transparente, dtype=np.uint8)
    rgb_array[matriz == 1] = cor
    return rgb_array

class Map:
    def __init__(self, image: str, width=400, height=400):

        # Dimensões do mapa
        self.width = width
        self.height = height

        # Informações da imagem
        img = Image.open(image).convert("L").resize((width, height))
        self.matriz = (np.array(img) < 128).astype(np.uint8)
        rgb_array = toRGBArray(self.matriz)

        # Superfice que será renderizada usando PyGame
        self.original_surface = pygame.surfarray.make_surface(rgb_array.transpose((1, 0, 2)))
        self.surface = self.original_surface

        # Espaço de configuração para diferentes estados do robô
        self.confSpace = []
        self.angleStep = 15
        self.last_index = 0

    def processMask(self, mask, angleStep):
        self.confSpace.clear()
        angles = range(0, 360, angleStep)
        self.angleStep = angleStep
        for ang in angles:
            maskRot = rotate(mask, ang, reshape=True, order=0)
            maskRot = (maskRot > 0.5).astype(np.uint8)
            conf = binary_dilation(self.matriz, structure=maskRot)
            self.confSpace.append(conf.astype(np.uint8))
    
    def getMatrix(self, angleRad=0, conf = True):
        if self.confSpace and conf:
            angleDeg = np.degrees(angleRad) % 180 
            index = int(angleDeg //  self.angleStep)
            index = index % len(self.confSpace)
            self.last_index = index
            return self.confSpace[index]
        else:
            return self.matriz
        
    def getSurfaceWithoutAngle(self):
        if self.confSpace:
            conf = self.confSpace[self.last_index]

            rgb_array = toRGBArray(conf)
            self.surface = pygame.surfarray.make_surface(rgb_array.transpose((1, 0, 2)))
        else:
            rgb_array = toRGBArray(self.matriz)
            self.surface = pygame.surfarray.make_surface(rgb_array.transpose((1, 0, 2)))
        return self.surface

    def getSurface(self, angleRad=0):
        if self.confSpace:
            angleDeg = np.degrees(angleRad) % 180 
            index = int(angleDeg //  self.angleStep)
            index = index % len(self.confSpace)
            conf = self.confSpace[index]

            rgb_array = toRGBArray(conf)
            self.surface = pygame.surfarray.make_surface(rgb_array.transpose((1, 0, 2)))
        else:
            rgb_array = toRGBArray(self.matriz)
            self.surface = pygame.surfarray.make_surface(rgb_array.transpose((1, 0, 2)))
        return self.surface

class Robot:
    def __init__(self, x=0, y=0, theta=0, width=36, height=35):
        self.x = x
        self.y = y
        self.theta = theta
        self.width = width
        self.height = height
        self.matriz_original = np.ones((height, width), dtype=np.uint8)
        self.updateSurface(theta)

    def updateSurface(self, theta):
        rotada = rotate(self.matriz_original, np.degrees(theta), reshape=True, order=0)
        self.surface = pygame.surfarray.make_surface(toRGBArrayColored(rotada, cor=(255, 0, 0)))
        self.surface.set_colorkey((255, 0, 255))

    def getSurface(self):
        return self.surface

    def updatePose(self, x=None, y=None, theta=None):
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y
        if theta is not None:
            self.theta = theta
            self.updateSurface(theta)

    def getMatrix(self, rotated=False):
        if rotated:
            ang = np.degrees(self.theta)
            rotada = rotate(self.matriz_original, ang, reshape=True, order=0)
            return (rotada > 0.5).astype(np.uint8)
        return self.matriz_original

class Pose:
    def __init__(self, x, y, theta = 0,color=(255, 0, 0), radius=5):
        self.x = x
        self.y = y
        self.color = color
        self.radius = radius
        self.theta = theta
        self.parent = None
        self.child_count = 0

    def move(self, dx, dy):
        self.x += dx
        self.y += dy

    def draw(self, surface, scale_x=1, scale_y=1):
        if self.parent is not None:
            pygame.draw.line(
                surface, 
                (0,0,255), 
                (int(self.parent.x * scale_x), int(self.parent.y * scale_y)),
                (int(self.x * scale_x), int(self.y * scale_y)),
                2
            )
        pygame.draw.circle(surface, self.color, (int(self.x * scale_x), int(self.y * scale_y)), self.radius)

class RRT:
    def __init__(self, step = 15, initial_pose = Pose(10, 10), goal_pose = Pose(750, 750, color=(0, 255, 0)), map: Map = None, obstacle_bias = False, obstacle_bias_param = [10,5,30]):
        self.initial_pose = initial_pose
        self.goal_pose = goal_pose
        self.start_tree:list[Pose] = [initial_pose]
        self.goal_tree: list[Pose] = [goal_pose]
        self.step = step
        self.map: Map = map
        self.surface = None
        self.tested_points = []
        self.last_tested_line = None
        self.goal_bias_threshold = 5
        self.leaf_bias_threshold = 80
        self.complete = False
        self.path = []
        self.alternate = True
        self.obstacle_bias = obstacle_bias
        self.obstacle_bias_param = obstacle_bias_param

        if self.obstacle_bias:
            dist = distance_transform_edt(1 - self.map.getMatrix(conf=False))

            self.weights = np.exp(-((dist - self.obstacle_bias_param[0]) ** 2) / (2 * self.obstacle_bias_param[1]**2))

            self.weights[self.map == 1] = 0  

            self.weights_flat = self.weights.flatten()
            self.weights_flat /= self.weights_flat.sum()

    
    def add_node(self, new_pose: Pose, parent: Pose, tree: list):
        new_pose.parent = parent
        parent.child_count += 1
        tree.append(new_pose)
        return new_pose
    
    def draw(self, surface):
        self.surface = surface
        self.goal_pose.draw(surface)
        for pose in self.start_tree:
            pose.draw(surface)
        for pose in self.goal_tree:
            pose.draw(surface)
        self.draw_path(surface)
    
    def reconstruct_path(self, node_from_start, node_from_goal):
        """ Reconstrói caminho a partir dos dois nós conectados """
        path_start = []
        current = node_from_start
        while current is not None:
            path_start.append(current)
            current = current.parent
        path_start.reverse()

        path_goal = []
        current = node_from_goal
        while current is not None:
            path_goal.append(current)
            current = current.parent

        self.path = path_start + path_goal
        print(f"Caminho encontrado com {len(self.path)} nós.")

    def draw_path(self, surface):
        if self.path and len(self.path) > 1:
            path_points = [(node.x, node.y) for node in self.path]
            pygame.draw.lines(surface, (0, 255, 0), False, path_points, 4)

    def get_path(self):
        return self.path

    def isPath(self, initial_pose, final_pose):
        dx = final_pose.x - initial_pose.x
        dy = final_pose.y - initial_pose.y
        angle = math.atan2(dy, dx)
        x = int(initial_pose.x + self.step * math.cos(angle))
        y = int(initial_pose.y + self.step * math.sin(angle))

        angle_step = math.radians(self.map.angleStep)
        start_ang = initial_pose.theta % (2 * math.pi)
        end_ang  = angle % (2 * math.pi)

        diff = (end_ang - start_ang + math.pi) % (2 * math.pi) - math.pi
        n_steps = max(1, int(abs(diff) / angle_step))

        for i in range(n_steps + 1):
            ang = start_ang + i * (diff / n_steps)
            map_start = self.map.getMatrix(ang)
            if not (0 <= x < map_start.shape[1] and 0 <= y < map_start.shape[0]):
                return None
            if map_start[y, x] == 1:
                return None

        map_matrix = self.map.getMatrix(angle)
        test_pose = Pose(x, y, color=(0, 255, 255)) 
        self.tested_points.append(test_pose)

        if x < 0 or x >= map_matrix.shape[1] or y < 0 or y >= map_matrix.shape[0]:
            return None
        if map_matrix[y, x] == 1:
            return None
        
        return test_pose

    def get_leaves(self, tree: list):
        if len(tree) < 5:
            return tree
        return [node for node in tree if node.child_count == 0]

    def draw_test_animation(self, surface):
        if self.last_tested_line:
            start_pos, end_pos = self.last_tested_line
            pygame.draw.line(surface, (255, 255, 0), 
                                 (int(start_pos.x), int(start_pos.y)), 
                                 (int(end_pos.x), int(end_pos.y)), 1)
        
        for pose in self.tested_points:
            pygame.draw.circle(surface, pose.color, (int(pose.x), int(pose.y)), 2)
            
    # --- NOVO MÉTODO ---
    def _check_collision_free_path(self, initial_pose: Pose, final_pose: Pose) -> bool:
        """Verifica se um caminho reto entre duas poses está livre de colisões."""
        dx = final_pose.x - initial_pose.x
        dy = final_pose.y - initial_pose.y
        distance = math.hypot(dx, dy)
        if distance == 0:
            return True

        angle = math.atan2(dy, dx)
        map_matrix = self.map.getMatrix(angle)

        steps = int(distance)
        if steps == 0:
            steps = 1

        step_dx = dx / steps
        step_dy = dy / steps

        for i in range(1, steps + 1):
            x = int(initial_pose.x + step_dx * i)
            y = int(initial_pose.y + step_dy * i)
            
            if not (0 <= y < map_matrix.shape[0] and 0 <= x < map_matrix.shape[1]):
                return False
            if map_matrix[y, x] == 1:
                return False
        return True

    # --- NOVO MÉTODO ---
    def smooth_short_circuit(self, gui_update_callback: Optional[Callable] = None):
        """Otimiza o caminho atual (`self.path`) criando atalhos."""
        if len(self.path) < 3:
            return

        print("Otimizando o caminho...")
        optimized_path = [self.path[0]]
        
        current_index = 0
        while current_index < len(self.path) - 1:
            best_next_index = current_index + 1
            # Itera de trás para frente para encontrar o "atalho" mais longo possível
            for lookahead_index in range(len(self.path) - 1, current_index + 1, -1):
                if self._check_collision_free_path(self.path[current_index], self.path[lookahead_index]):
                    best_next_index = lookahead_index
                    break 
            
            optimized_path.append(self.path[best_next_index])
            current_index = best_next_index

        original_nodes = len(self.path)
        self.path = optimized_path
        optimized_nodes = len(self.path)
        print(f"Caminho otimizado de {original_nodes} para {optimized_nodes} nós.")

        if gui_update_callback:
            gui_update_callback()

    def tick(self):
        if self.complete:
            return

        self.tested_points.clear()
        self.last_tested_line = None

        if self.alternate:
            tree_from = self.start_tree
            tree_to  = self.goal_tree
        else:
            tree_from = self.goal_tree
            tree_to  = self.start_tree
        self.alternate = not self.alternate

        new_x = random.randint(0, self.map.width)
        new_y = random.randint(0, self.map.height)
        new_theta = random.uniform(0, 2 * math.pi)
        new_pose = Pose(new_x, new_y, new_theta)

        choice = random.randint(0,100)
        if choice <= self.goal_bias_threshold:
            new_pose = random.choice(tree_to) 
        elif choice <= self.obstacle_bias_param[2] and self.obstacle_bias == True:
            indice = np.random.choice(len(self.weights_flat), p=self.weights_flat)
            y, x = np.unravel_index(indice, self.weights.shape)
            new_theta = random.uniform(0, 2 * math.pi)
            new_pose = Pose(int(x), int(y), new_theta)

        if choice <= self.leaf_bias_threshold:
            nearest = min(self.get_leaves(tree_from), key=lambda pose: math.hypot(new_pose.x - pose.x, new_pose.y - pose.y))
        else:
            nearest = min(tree_from, key=lambda pose: math.hypot(new_pose.x - pose.x, new_pose.y - pose.y))

        self.last_tested_line = (nearest, new_pose)
        test_pose = self.isPath(nearest, new_pose)

        if test_pose is not None:
            added_node = self.add_node(test_pose, nearest, tree_from)

            nearest_other = min(tree_to, key=lambda pose: math.hypot(test_pose.x - pose.x, test_pose.y - pose.y))
            dist = math.hypot(added_node.x - nearest_other.x, added_node.y - nearest_other.y)
            if dist <= self.step:
                self.complete = True
                if tree_from is self.start_tree:
                    node_from_start = added_node
                    node_from_goal = nearest_other
                else:
                    node_from_start = nearest_other
                    node_from_goal = added_node

                self.reconstruct_path(node_from_start, node_from_goal)

def run_rrt(
    image_path: str,
    start: Tuple[float, float, float],
    goal: Tuple[float, float, float],
    robot_size: Tuple[int, int] = (36, 35),
    step: int = 15,
    angleStep: int = 15,
    max_iterations: int = 20000,
    gui: bool = False,
    progress_callback: Optional[Callable[[List[Pose], List[Pose], Optional[Tuple[Pose, Pose]]], None]] = None,
    timeout: Optional[float] = None,
    obstacle_bias = False,
    obstacle_bias_params = [10,5,50],
) -> Optional[List[Pose]]:
    """Executa RRT e retorna a lista de Pose do caminho encontrado (ou None)."""
    if gui:
        pygame.init()

    MAP_WIDTH = 800
    MAP_HEIGHT = 800
    m = Map(image_path, width=MAP_WIDTH, height=MAP_HEIGHT)
    robot = Robot(width=robot_size[0], height=robot_size[1])
    m.processMask(robot.getMatrix(rotated=True), angleStep)

    start_pose = Pose(start[0], start[1], start[2])
    goal_pose = Pose(goal[0], goal[1], goal[2], color=(0,255,0))

    rrt = RRT(step=step, initial_pose=start_pose, goal_pose=goal_pose, map=m, obstacle_bias=obstacle_bias, obstacle_bias_param=obstacle_bias_params)

    start_time = time.time()
    iterations = 0

    if gui:
        screen = pygame.display.set_mode((MAP_WIDTH, MAP_HEIGHT))
        pygame.display.set_caption("RRT Module")
        clock = pygame.time.Clock()
        running = True
    else:
        screen = None
        clock = None
        running = True

    waiting = True

    while running:
        if gui:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

        while waiting:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        waiting = False
        rrt.tick()
        iterations += 1

        if progress_callback:
            try:
                progress_callback(rrt.get_path(), rrt.tested_points, rrt.last_tested_line)
            except Exception:
                pass

        if gui and screen is not None:
            screen.fill((255,255,255))
            screen.blit(m.getSurfaceWithoutAngle(), (0,0))
            rrt.draw(screen)
            rrt.draw_test_animation(screen)
            pygame.display.flip()
            clock.tick(120)

        if rrt.complete or iterations >= max_iterations or (timeout and (time.time() - start_time) > timeout):
            running = False

    if rrt.complete:
        
        gui_update_callback = None
        if gui:
            def update_screen_cspace():
                if screen is None: return
                screen.fill((255, 255, 255))
                screen.blit(m.getSurfaceWithoutAngle(), (0, 0)) # Mostra C-Space
                rrt.draw(screen)
                pygame.display.flip()
                pygame.event.pump()
            
            update_screen_cspace()
            time.sleep(0.5) 
            
            gui_update_callback = update_screen_cspace
        
        rrt.smooth_short_circuit(gui_update_callback=gui_update_callback)
        
        if gui:
            # --- MODIFICADO AQUI ---
            # Nova função para desenhar o resultado final sobre o mapa original
            def draw_final_view():
                if screen is None: return
                screen.fill((255, 255, 255))
                # Usa a superfície do mapa original
                screen.blit(m.original_surface, (0, 0)) 
                rrt.draw(screen) # Desenha a árvore e o caminho otimizado
                pygame.display.flip()
                pygame.event.pump()

            # Loop final que mostra o caminho no mapa original
            end_time = time.time() + 5
            while time.time() < end_time:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        end_time = 0
                draw_final_view() # Chama a nova função de desenho
                if clock: clock.tick(30)
    
    # if gui:
    #     pygame.quit()

    path = rrt.get_path()
    return path if path else None
# ----------------------------- helper export --------------------------------
__all__ = ['run_rrt', 'Pose', 'Map', 'Robot']
