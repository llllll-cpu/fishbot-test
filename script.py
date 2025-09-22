#!/usr/bin/env python3
import os
from PIL import Image
import yaml

# ================== 配置 ==================
yaml_file = "/home/ros/fishbot/src/fishbot_navigation2/maps/map.yaml"
world_file = "/home/ros/fishbot/src/fishbot_description/world/chejian_world.world"
wall_height = 2.0  # 墙体高度（米）
# ========================================

# 读取 YAML
with open(yaml_file, 'r') as f:
    map_data = yaml.safe_load(f)

# 处理 PGM 文件路径
image_file = map_data['image']
if not os.path.isabs(image_file):
    image_file = os.path.join(os.path.dirname(yaml_file), image_file)
image_file = os.path.expanduser(image_file)

if not os.path.exists(image_file):
    raise FileNotFoundError(f"PGM 文件不存在: {image_file}")

resolution = map_data['resolution']
origin = map_data['origin']  # [x, y, yaw]

# 读取 PGM
im = Image.open(image_file)
pixels = im.load()
width, height = im.size

# 写入 Gazebo world 文件
with open(world_file, "w") as f:
    f.write('<world name="generated_world">\n')
    f.write('  <include><uri>model://ground_plane</uri></include>\n')
    f.write('  <include><uri>model://sun</uri></include>\n')

    # 遍历像素按行合并连续黑色像素生成一条墙
    for y in range(height):
        x = 0
        while x < width:
            if pixels[x, y] < 127:
                start_x = x
                while x < width and pixels[x, y] < 127:
                    x += 1
                end_x = x - 1

                # 计算墙体中心位置和长度
                wall_x = origin[0] + ((start_x + end_x) / 2) * resolution
                wall_y = origin[1] + (height - y) * resolution
                wall_length = (end_x - start_x + 1) * resolution

                # 写入模型
                f.write(f'''
  <model name="wall_{y}_{start_x}_{end_x}">
    <pose>{wall_x} {wall_y} {wall_height/2} 0 0 0</pose>
    <link name="link">
      <collision>
        <geometry>
          <box><size>{wall_length} {resolution} {wall_height}</size></box>
        </geometry>
      </collision>
      <visual>
        <geometry>
          <box><size>{wall_length} {resolution} {wall_height}</size></box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Grey</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
                ''')
            else:
                x += 1

    f.write('</world>\n')

print(f"Gazebo world 已生成: {world_file}")
