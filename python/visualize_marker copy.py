import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 좌표 예시 (m)
base_pos = [0.0, 0.0, 0.0]          # base_link 위치
camera_pos = [0.0, 0.0, 0.2]        # 카메라 높이 0.2 m
marker_pos = [-0.284, -0.144, 0.377]  # ArUco 마커 위치

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# base_link
ax.scatter(base_pos[0], base_pos[1], base_pos[2], c='green', s=100, label='base_link')

# camera
ax.scatter(camera_pos[0], camera_pos[1], camera_pos[2], c='blue', s=100, label='Camera')

# marker
ax.scatter(marker_pos[0], marker_pos[1], marker_pos[2], c='red', s=100, label='Marker ID 6')

# base_link -> camera
ax.plot([base_pos[0], camera_pos[0]],
        [base_pos[1], camera_pos[1]],
        [base_pos[2], camera_pos[2]], 'g--', label='base -> camera')

# camera -> marker
ax.plot([camera_pos[0], marker_pos[0]],
        [camera_pos[1], marker_pos[1]],
        [camera_pos[2], marker_pos[2]], 'b--', label='camera -> marker')

# 축 레이블
ax.set_xlabel('X (Right)')
ax.set_ylabel('Y (Forward)')
ax.set_zlabel('Z (Up)')

# 축 범위
ax.set_xlim(-0.5, 0.5)
ax.set_ylim(-0.5, 0.5)
ax.set_zlim(0, 0.5)

ax.legend()
ax.set_title("TF: base_link -> camera -> ArUco Marker")

plt.show()
