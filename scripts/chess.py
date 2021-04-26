#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time
from copy import deepcopy
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from robotis_controller_msgs.msg import SyncWriteItem
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState, Constraints


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonInteface(object):
  """MoveGroupPythonInteface"""
  def __init__(self):

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    # Create a publisher for the real ROBOTIS gripper
    robotis_publisher = rospy.Publisher('/robotis/direct/sync_write_item', SyncWriteItem, queue_size=5)

    # And create another one for the Gazebo simulated gripper
    gazebo_publisher = rospy.Publisher('/gripper_gazebo_controller/command', JointTrajectory, queue_size=1)


    # Chess steps subscriber
    self.subscribe_ena = False
    rospy.Subscriber("chess_steps", String, self.chess_step_callback)


    # Getting Basic Information
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print ""

    self.constraints = Constraints()

    # Robotis gripper
    self.goal_position_msg = SyncWriteItem()
    self.goal_position_msg.item_name = "goal_position"
    self.goal_position_msg.joint_name = ["gripper"]

    # Gazebo gripper
    self.gazebo_trajectory_command = JointTrajectory()
    self.gazebo_trajectory_command.joint_names = ["gripper"]
    self.gazebo_trajectory_point = JointTrajectoryPoint()
    self.gazebo_trajectory_point.time_from_start = rospy.rostime.Duration(1,0)
    self.gazebo_trajectory_point.velocities = [0.0]

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.robotis_publisher = robotis_publisher
    self.gazebo_publisher = gazebo_publisher

    # Chess related variables
    # Y coordinate
    self.rows = {"1": 0.420, "2": 0.393, "3": 0.366, "4": 0.339, "5": 0.311, "6": 0.284, "7": 0.257, "8": 0.230}
    # X coordinate
    self.columns  = {"a": 0.15, "b": 0.123, "c": 0.096, "d": 0.069, "e": 0.041, "f": 0.014, "g": -0.013, "h": -0.040}
    self.z_high = 0.222
    self.z_low = 0.17
    self.z_drop = 0.172
    self.z_touch_table = 0.165
    self.z_drop_to_table = 0.16
    self.x_drop_to_table = -0.1
    self.y_drop_to_table = 0.3

  def enable_subscribe(self):
    self.subscribe_ena = True

  def chess_step_callback(self, data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    if self.subscribe_ena:
      (steps, hit) = data.data.split(";")
      print(steps)
      print(hit)
      if hit == "True": hit = True
      else: hit = False
      self.do_chess_step(steps[:2], steps[2:], hit)



  def set_gripper(self, status):
      if status == "open":
        # Real gripper value
        self.goal_position_msg.value = [620]
        # Gazebo gripper value
        self.gazebo_trajectory_point.positions = [0.7]
      else:
        self.goal_position_msg.value = [740]
        self.gazebo_trajectory_point.positions = [1.135]

      # Publish real gripper position
      self.robotis_publisher.publish(self.goal_position_msg)

      # Publish gazebo gripper position
      self.gazebo_trajectory_command.header.stamp = rospy.Time.now()
      self.gazebo_trajectory_command.points = [self.gazebo_trajectory_point]
      self.gazebo_publisher.publish(self.gazebo_trajectory_command)


  def do_chess_step(self, start, end, hit=False):

      # 0) Make sure that the gripper is open
      self.set_gripper("open")

      # 1) If it's a hit:
      if hit:
        # 1.1) Go above end position
        self.go_to_pose_goal(self.columns[end[0]], self.rows[end[1]], self.z_high)
        time.sleep(0.2)

        # 1.2) Go down
        self.go_to_pose_goal(self.columns[end[0]], self.rows[end[1]], self.z_low)
        time.sleep(0.2)

        # 1.3) Grab the figure
        self.set_gripper("closed")
        time.sleep(0.2)

        # 1.4) Move up
        self.go_to_pose_goal(self.columns[end[0]], self.rows[end[1]], self.z_high)
        time.sleep(0.2)

        # 1.5) Go out of the chess table
        self.go_to_pose_goal(self.x_drop_to_table, self.y_drop_to_table, self.z_high)
        time.sleep(0.2)

        # 1.6) Go down
        self.go_to_pose_goal(self.x_drop_to_table, self.y_drop_to_table, self.z_drop_to_table)
        time.sleep(0.2)

        # 1.7) Release the figure
        self.set_gripper("open")
        time.sleep(0.2)

        # 1.8) Move up
        self.go_to_pose_goal(self.x_drop_to_table, self.y_drop_to_table, self.z_high)
        time.sleep(0.2)

      # 2) Go above start position
      self.go_to_pose_goal(self.columns[start[0]], self.rows[start[1]], self.z_high)
      time.sleep(0.2)

      # 3) Go down
      self.go_to_pose_goal(self.columns[start[0]], self.rows[start[1]], self.z_low)
      time.sleep(0.2)

      # 4) Grab the figure
      self.set_gripper("closed")
      time.sleep(0.2)

      # 5) Move up
      self.go_to_pose_goal(self.columns[start[0]], self.rows[start[1]], self.z_high)
      time.sleep(0.2)

      # 6) Go above end position
      self.go_to_pose_goal(self.columns[end[0]], self.rows[end[1]], self.z_high)
      time.sleep(0.2)

      # 7) Move down
      self.go_to_pose_goal(self.columns[end[0]], self.rows[end[1]], self.z_drop)
      time.sleep(0.2)

      # 8) Open gripper
      self.set_gripper("open")
      time.sleep(0.2)

      # 9) Move up
      self.go_to_pose_goal(self.columns[end[0]], self.rows[end[1]], self.z_high)
      time.sleep(0.2)

      # 10) Go home
      self.go_to_home()
      time.sleep(0.2)


  def go_to_home(self):

    ## Planning to a Joint Goal
    ## The UR's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = -1.5708
    joint_goal[1] = -1.5708
    joint_goal[2] = -1.0472
    joint_goal[3] = -1.0472
    joint_goal[4] = 1.5708
    joint_goal[5] = 0.7854

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    self.move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()

    # Open gripper
    self.set_gripper("open")

    # For testing:
    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self, x, y, z):
    ## Planning to a Pose Goal
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    # set proper quaternion for the vertical orientation: https://quaternions.online/
    pose_goal.orientation.x = -0.383
    pose_goal.orientation.y = 0.924
    
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    self.move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = self.move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def do_calibration(self):
    raw_input("============ Press `Enter` to go A8 (down) ...")
    self.set_gripper("closed")
    self.go_to_pose_goal(x = self.columns["a"], y = self.rows["8"], z = self.z_touch_table)
    raw_input("============ Press `Enter` to go D8 (down) ...")
    self.go_to_pose_goal(x = self.columns["d"], y = self.rows["8"], z = self.z_touch_table)
    raw_input("============ Press `Enter` to go E8 (down) ...")
    self.go_to_pose_goal(x = self.columns["e"], y = self.rows["8"], z = self.z_touch_table)
    raw_input("============ Press `Enter` to go H8 (down) ...")
    self.go_to_pose_goal(x = self.columns["h"], y = self.rows["8"], z = self.z_touch_table)
    raw_input("============ Press `Enter` to go H1 (down) ...")
    self.go_to_pose_goal(x = self.columns["h"], y = self.rows["1"], z = self.z_touch_table)
    raw_input("============ Press `Enter` to go A1 (down) ...")
    self.go_to_pose_goal(x = self.columns["a"], y = self.rows["1"], z = self.z_touch_table)
    raw_input("============ Press `Enter` to go A8 (down) ...")
    self.go_to_pose_goal(x = self.columns["a"], y = self.rows["8"], z = self.z_touch_table)
    raw_input("============ Press `Enter` to go A8 (up) ...")
    self.go_to_pose_goal(x = self.columns["a"], y = self.rows["8"], z = self.z_high)
    raw_input("============ Press `Enter` to go D8 (up) ...")
    self.go_to_pose_goal(x = self.columns["d"], y = self.rows["8"], z = self.z_high)
    raw_input("============ Press `Enter` to go E8 (up) ...")
    self.go_to_pose_goal(x = self.columns["e"], y = self.rows["8"], z = self.z_high)
    raw_input("============ Press `Enter` to go H8 (up) ...")
    self.go_to_pose_goal(x = self.columns["h"], y = self.rows["8"], z = self.z_high)
    raw_input("============ Press `Enter` to go H1 (up) ...")
    self.go_to_pose_goal(x = self.columns["h"], y = self.rows["1"], z = self.z_high)
    raw_input("============ Press `Enter` to go A1 (up) ...")
    self.go_to_pose_goal(x = self.columns["a"], y = self.rows["1"], z = self.z_high)
    raw_input("============ Press `Enter` to go home ...")
    self.go_to_home()

  def display_trajectory(self, plan):
    ## Displaying a Trajectory
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    self.display_trajectory_publisher.publish(display_trajectory)


  def execute_plan(self, plan):
    ## Executing a Plan
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    self.move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

def main():
  try:

    moveit_commander = MoveGroupPythonInteface()

    # Set max velocity
    moveit_commander.move_group.set_max_velocity_scaling_factor(0.2)
    # Set tolerances, without that IK cannot do a valid plan
    moveit_commander.move_group.set_goal_position_tolerance(0.0005)
    moveit_commander.move_group.set_goal_orientation_tolerance(0.001)
    
    raw_input("============ Press `Enter` to go home...")
    moveit_commander.go_to_home()
    while 1:
      print("============ Select mode:")
      print("============   p: Play")
      print("============   c: Calibration")
      print("============   s: Subscribe to chess_steps topic")
      ret = raw_input()
      if ret in ["p", "c", "s"]:
        break
      else:
        pass

    if ret == "c":
      moveit_commander.do_calibration()

    elif ret == "p":
      raw_input("============ Press `Enter` to start playing...")
      moveit_commander.do_chess_step("d7", "d5")
      raw_input("============ Press `Enter` for the next step...")
      moveit_commander.do_chess_step("e7", "e6")
      raw_input("============ Press `Enter` for the next step...")
      moveit_commander.do_chess_step("g8", "f6")
      raw_input("============ Press `Enter` for the next step...")
      moveit_commander.do_chess_step("f8", "b4")
      raw_input("============ Press `Enter` for the next step...")
      moveit_commander.do_chess_step("e8", "a4")
      raw_input("============ Press `Enter` for the next step...")
      moveit_commander.do_chess_step("b4", "c3", hit = True)

    elif ret == "s":
      moveit_commander.enable_subscribe()
      rospy.spin()
    

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
