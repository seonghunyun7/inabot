# laser object detect
sudo apt install ros-humble-visualization-msgs
sudo apt install ros-humble-message-filters
sudo apt install ros-humble-vision-msgs
sudo apt install ros-humble-pcl-conversions



LaserScan 입력
       │
       ▼
  이상치 제거
(removeOutliers)
       │
       ▼
LaserScan → PointCloud2 변환
(transformLaserScanToPointCloud)
       │
       ▼
   ROI 필터링
(filterPointCloud)
       │
       ▼
  ROI 안 포인트만 사용
       │
       ▼
  최소 거리 계산
       │
       ▼
  최소 거리 발행
(obstacle_dist_pub_)
       │
       ▼
  최소 거리 반환



           ┌─────────────────────┐
           │   LaserScan 입력    │
           └─────────┬───────────┘
                     │
                     ▼
         ┌─────────────────────┐
         │  이상치 제거         │
         │ removeOutliers()    │
         └─────────┬───────────┘
                     │
                     ▼
         ┌─────────────────────┐
         │ LaserScan → PointCloud2 │
         │ transformLaserScanToPointCloud() │
         └─────────┬───────────┘
                     │
                     ▼
         ┌─────────────────────┐
         │  ROI 필터링          │
         │ filterPointCloud()  │
         │ (로봇 전방 기준)    │
         └─────────┬───────────┘
                     │
                     ▼
         ┌─────────────────────┐
         │  ROI 안 포인트만 사용 │
         │  최소 거리 계산       │
         │  (min_distance)     │
         └─────────┬───────────┘
                     │
                     ▼
         ┌─────────────────────┐
         │ 거리 발행 & 반환      │
         │ obstacle_dist_pub_  │
         │ return min_distance │
         └─────────────────────┘


           Y
           │
           │
           │
           │       장애물
           │       ■
           │
           │
   ┌───────┴─────────┐
   │                   │
   │   로봇 바디 : 로봇 중심        │
   │                   │
   └───────┬─────────┘
           │
           │ robot_front_x_  ← 로봇 앞끝 기준
           ▼
      ROI 시작점
           │
           │
           ▼
      최소 거리 = min_distance : ROI 안 가장 가까운 장애물까지 거리 → 따라서 로봇 전방 끝 기준 거리
           │
           │
       토픽 데이터로 발행


 

 