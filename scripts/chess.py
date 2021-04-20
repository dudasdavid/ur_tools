#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

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

## END_SUB_TUTORIAL


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


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

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

    robotis_publisher = rospy.Publisher('/robotis/direct/sync_write_item', SyncWriteItem, queue_size=5)

    gazebo_publisher = rospy.Publisher('/gripper_gazebo_controller/command', JointTrajectory, queue_size=1)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
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
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

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


  def do_chess_step(self, start, end):
      # Y coordinate
      rows = {"1": 0.420, "2": 0.393, "3": 0.366, "4": 0.339, "5": 0.311, "6": 0.284, "7": 0.257, "8": 0.230, "X": 0.3}
      # X coordinate
      columns  = {"A": 0.15, "B": 0.123, "C": 0.096, "D": 0.069, "E": 0.041, "F": 0.014, "G": -0.013, "H": -0.040, "X": -0.1}

      z_high = 0.25
      z_low = 0.17
      z_drop = 0.177

      # 1) Go above start position
      self.set_gripper("open")
      self.go_to_pose_goal(columns[start[0]], rows[start[1]], z_high)
      time.sleep(0.2)

      # 2) Go down
      #self.go_to_pose_goal(columns[start[0]], rows[start[1]], z_high - (z_high - z_low)/2)
      #time.sleep(0.1)
      self.go_to_pose_goal(columns[start[0]], rows[start[1]], z_low)
      time.sleep(0.1)

      # 3) Grab the figure
      self.set_gripper("closed")
      time.sleep(0.2)

      # 4) Move up
      #self.go_to_pose_goal(columns[start[0]], rows[start[1]], z_high - (z_high - z_low)/2)
      #time.sleep(0.1)
      self.go_to_pose_goal(columns[start[0]], rows[start[1]], z_high)
      time.sleep(0.1)

      # 5) Go above end position
      self.go_to_pose_goal(columns[end[0]], rows[end[1]], z_high)
      time.sleep(0.2)

      # 6) Move down
      #self.go_to_pose_goal(columns[end[0]], rows[end[1]], z_high - (z_high - z_drop)/2)
      #time.sleep(0.1)
      self.go_to_pose_goal(columns[end[0]], rows[end[1]], z_drop)
      time.sleep(0.1)

      # 7) Open gripper
      self.set_gripper("open")
      time.sleep(0.2)

      # 8) Move up
      #self.go_to_pose_goal(columns[end[0]], rows[end[1]], z_high - (z_high - z_drop)/2)
      #time.sleep(0.1)
      self.go_to_pose_goal(columns[end[0]], rows[end[1]], z_high)
      time.sleep(0.1)

      # 9) Go home if it's not a hit
      if end != "XX":
        self.go_to_home()
      time.sleep(0.2)


  def go_to_home(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = -1.5708
    joint_goal[1] = -1.5708
    joint_goal[2] = -1.0472
    joint_goal[3] = -1.0472
    joint_goal[4] = 1.5708
    joint_goal[5] = 0.7854


    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    ## END_SUB_TUTORIAL
    self.set_gripper("open")

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self, x, y, z):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    # set proper quaternion: https://quaternions.online/
    pose_goal.orientation.x = -0.383
    pose_goal.orientation.y = 0.924
    

    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL


def main():
  try:

    tutorial = MoveGroupPythonIntefaceTutorial()

    # Set max velocity
    tutorial.move_group.set_max_velocity_scaling_factor(0.5)
    # Set tolerances, without that IK cannot do a valid plan
    tutorial.move_group.set_goal_position_tolerance(0.001)
    tutorial.move_group.set_goal_orientation_tolerance(0.005)
    
    print "============ Press `Enter` to go home ..."
    raw_input()
    tutorial.go_to_home()

    print "============ Press `Enter` to start playing..."
    raw_input()
    tutorial.do_chess_step("D7", "D5")
    raw_input()
    tutorial.do_chess_step("E7", "E6")
    raw_input()
    tutorial.do_chess_step("G8", "F6")
    raw_input()
    tutorial.do_chess_step("F8", "B4")
    raw_input()
    tutorial.do_chess_step("E8", "A4")
    raw_input()
    tutorial.do_chess_step("C3", "XX")
    tutorial.do_chess_step("B4", "C3")
    

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
