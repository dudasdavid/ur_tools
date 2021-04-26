# ur_tools


Bringup in Gazebo
roslaunch ur_e_gazebo ur3e.launch limited:=true world_file:='$(find ur_tools)/worlds/world.world' z:=1.02
roslaunch ur3_e_moveit_config ur3_e_moveit_planning_execution.launch sim:=true limited:=true

rosrun ur_tools chess.py