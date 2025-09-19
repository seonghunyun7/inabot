Sick Safevisionary ROS2

git clone https://github.com/SICKAG/sick_safevisionary_ros2.git
git clone https://github.com/SICKAG/sick_safevisionary_base.git
rosdep install --from-paths ./ --ignore-src -y
cd ..
colcon build --packages-select sick_safevisionary_base sick_safevisionary_interfaces sick_safevisionary_driver  --cmake-args -DCMAKE_BUILD_TYPE=Release


ros2 launch sick_safevisionary_driver driver_node.launch.py