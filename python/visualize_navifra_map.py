import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ================================
# 1. JSON 파일 읽기
# ================================
file_path = "/home/ysh/inabot_ws/python/map_2_Navifra.json"
with open(file_path, "r") as f:
    map_data = json.load(f)

occupied_space = map_data.get("occupied_space", [])
if not occupied_space:
    print("occupied_space 데이터가 없습니다.")
    exit()

# ================================
# 2. 좌표 분리
# ================================
x_coords = [p[0] for p in occupied_space]
y_coords = [p[1] for p in occupied_space]
z_coords = [p[2] for p in occupied_space]  # 대부분 0

# ================================
# 3. Grid Map 생성
# ================================
x_min, x_max = min(x_coords), max(x_coords)
y_min, y_max = min(y_coords), max(y_coords)
resolution = map_data.get("resolution", 0.05)

grid_width = int(np.ceil((x_max - x_min) / resolution)) + 1
grid_height = int(np.ceil((y_max - y_min) / resolution)) + 1
grid = np.zeros((grid_height, grid_width))

for x, y in zip(x_coords, y_coords):
    ix = int((x - x_min) / resolution)
    iy = int((y - y_min) / resolution)
    if 0 <= ix < grid_width and 0 <= iy < grid_height:
        grid[iy, ix] = 1  # occupancy 확률 1로 설정

# ================================
# 4. Visualization
# ================================
fig = plt.figure(figsize=(18, 6))

# Topology Map (2D)
ax0 = fig.add_subplot(1, 3, 1)
ax0.scatter(x_coords, y_coords, c='black', s=10)
ax0.set_title("Topology Map (2D)")
ax0.set_xlabel("X [m]")
ax0.set_ylabel("Y [m]")
ax0.axis('equal')
ax0.grid(True)

# Topology Map (3D)
ax1 = fig.add_subplot(1, 3, 2, projection='3d')
ax1.scatter(x_coords, y_coords, z_coords, c='blue', s=10)
ax1.set_title("Topology Map (3D)")
ax1.set_xlabel("X [m]")
ax1.set_ylabel("Y [m]")
ax1.set_zlabel("Z [m]")
ax1.set_box_aspect([1,1,0.1])  # z가 작으므로 평면 비율 맞춤

# Grid Map
ax2 = fig.add_subplot(1, 3, 3)
im = ax2.imshow(grid, origin='lower', cmap='Greys', 
                extent=[x_min, x_max, y_min, y_max],
                interpolation='nearest')
ax2.set_title("Grid Map")
ax2.set_xlabel("X [m]")
ax2.set_ylabel("Y [m]")
ax2.axis('equal')
ax2.grid(True)
plt.colorbar(im, ax=ax2, fraction=0.046, pad=0.04, label="Occupancy")

plt.tight_layout()
plt.show()
