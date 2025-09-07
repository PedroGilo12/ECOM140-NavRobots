import sys
from PIL import Image

# Parâmetros
WALL_HEIGHT = 0.3      # Altura da parede (30 cm)
RESOLUTION = 0.05      # Tamanho de cada célula/pixel em metros (5 cm)
WORLD_NAME = "mapa_mundo"

def generate_optimized_sdf(image_path, sdf_path):
    """
    Gera um arquivo SDF otimizado agrupando pixels adjacentes em caixas maiores.
    """
    img = Image.open(image_path).convert("L")  # Converte para escala de cinza
    pixels = img.load()
    width, height = img.size

    # Cria uma grade para marcar pixels já visitados e evitar trabalho duplicado
    visited = [[False for _ in range(width)] for _ in range(height)]
    
    models = []
    model_count = 0

    # Percorre todos os pixels para encontrar o início de novas paredes
    for y in range(height):
        for x in range(width):
            # Se o pixel for preto e ainda não foi parte de outra parede
            if pixels[x, y] < 128 and not visited[x][y]:
                
                # --- Encontra a largura máxima do retângulo a partir de (x, y) ---
                rect_width = 1
                while (x + rect_width < width and 
                       pixels[x + rect_width, y] < 128 and 
                       not visited[x + rect_width][y]):
                    rect_width += 1
                
                # --- Encontra a altura máxima, verificando se a largura se mantém ---
                rect_height = 1
                end_of_rect = False
                while y + rect_height < height and not end_of_rect:
                    # Verifica se a linha inteira abaixo tem a mesma largura e é preta
                    for i in range(rect_width):
                        if pixels[x + i, y + rect_height] >= 128 or visited[x + i][y + rect_height]:
                            end_of_rect = True
                            break
                    if not end_of_rect:
                        rect_height += 1

                # Marca todos os pixels deste retângulo como visitados
                for i in range(rect_width):
                    for j in range(rect_height):
                        visited[x + i][y + j] = True

                # Calcula as dimensões e a posição do box no mundo
                size_x = rect_width * RESOLUTION
                size_y = rect_height * RESOLUTION
                
                # A posição é o centro do retângulo
                pos_x = (x + rect_width / 2 - width / 2) * RESOLUTION
                pos_y = (height / 2 - (y + rect_height / 2)) * RESOLUTION
                pos_z = WALL_HEIGHT / 2

                # Cria o SDF para este modelo único e maior
                model_sdf = f"""
    <model name="wall_{model_count}">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{size_x} {size_y} {WALL_HEIGHT}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{size_x} {size_y} {WALL_HEIGHT}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.2 1</ambient>
            <diffuse>0.2 0.2 0.2 1</diffuse>
          </material>
        </visual>
        <pose>{pos_x} {pos_y} {pos_z} 0 0 0</pose>
      </link>
    </model>"""
                models.append(model_sdf)
                model_count += 1

    # Calcula as dimensões totais do chão em metros
    ground_plane_width = 10
    ground_plane_height = 10

    # Monta o arquivo SDF final, agora com luz e chão do tamanho da imagem
    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="{WORLD_NAME}">
    
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin
      filename="gz-sim-imu-system"
      name="gz::sim::systems::Imu">
    </plugin>

    <!-- Luz direcional (sol) -->
    <light name='sun' type='directional'>
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Plano de chão com o tamanho exato da imagem -->
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{ground_plane_width} {ground_plane_height}</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{ground_plane_width} {ground_plane_height}</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Modelos das paredes gerados a partir da imagem -->
    {''.join(models)}

  </world>
</sdf>
"""

    with open(sdf_path, "w") as f:
        f.write(sdf_content)
    
    print(f"[OK] Mundo otimizado com {model_count} modelos salvo em {sdf_path}")

def main():
    if len(sys.argv) != 3:
        print("Uso: img2sdf entrada.png saida.sdf")
        sys.exit(1)
    generate_optimized_sdf(sys.argv[1], sys.argv[2])

if __name__ == '__main__':
    main()