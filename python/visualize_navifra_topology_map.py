import json
import matplotlib.pyplot as plt

# ================================
# 1. JSON 파일 읽기
# ================================
file_path = "/home/ysh/inabot_ws/python/map_2_Navifra.json"
with open(file_path, "r") as f:
    map_data = json.load(f)

# ================================
# 2. occupied_space 데이터 추출
# ================================
occupied_space = map_data.get("occupied_space", [])

# 데이터 없으면 경고
if not occupied_space:
    print("occupied_space 데이터가 없습니다.")
    exit()

# ================================
# 3. x, y 좌표 분리
# ================================
x_coords = [p[0] for p in occupied_space]
y_coords = [p[1] for p in occupied_space]

# ================================
# 4. 2D 맵 시각화
# ================================
plt.figure(figsize=(8, 8))
plt.scatter(x_coords, y_coords, c='black', s=10)  # 점으로 표시
plt.title("Occupied Space Map")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.axis('equal')  # x, y 비율 동일하게
plt.grid(True)
plt.show()
