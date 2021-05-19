# ur_tools


Bringup in Gazebo
roslaunch ur_e_gazebo ur3e.launch limited:=true world_file:='$(find ur_tools)/worlds/world.world' z:=1.02
roslaunch ur3_e_moveit_config ur3_e_moveit_planning_execution.launch sim:=true limited:=true

Bringup real robot:
roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=10.0.0.244 limited:=true
roslaunch rh_p12_rn_a_tools bringup.launch

roslaunch ur3_e_moveit_config ur3_e_moveit_planning_execution.launch limited:=true
roslaunch ur3_e_moveit_config moveit_rviz.launch config:=true

rosrun ur_tools chess.py


mandatory ROS packages after fresh install (Noetic):
sudo apt install socat
sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-ur-client-library
sudo apt install ros-noetic-joint-trajectory-controller
sudo apt install ros-noetic-moveit
sudo apt install ros-noetic-industrial-robot-status-interface
sudo apt install ros-noetic-velocity-controllers
sudo apt install ros-noetic-force-torque-sensor-controller

git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework-msgs
git clone https://github.com/MOGI-ROS/gazebo_ros_link_attacher