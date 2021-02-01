#!/usr/bin/env python

""" Script Writer: Yulin Wang | UUN: S1889038 """
""" - this script controls the robot to draw a curve in 2D"""
""" Purpose of code: Simulate the trajectory of robotic arm, thereby getting to know Gazebo """

"""

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions
are met:

1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

3. Neither the name of SRI International nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission

"""

""" --------- Step One - Install ROS, Prepare RViz and Moveit Interfaces will be using --------- """

""" To use the Python MoveIt interfaces, 'moveit_commander' _ namespace will be imported first
    This namespace provides us with a 'MoveGroupCommander' _ class, a 'PlanningSceneInterface' _ class,
    and a 'RobotCommander'_ class. More on these below."""
""" --------- Step Two - Import all the required packages and some other messages that will be used """
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


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


class roboticprintingcontrol(object):
  """ Move the group of joints using python """
  def __init__(self):
    super(roboticprintingcontrol, self).__init__()

    # First initialize 'moveit_commander' and a 'rospy' node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robotic_printing_control', anonymous=True)

    # Instantiate a "RobotCommander" object. Provides information such as the robot's
    # kinematic model and the robot's current joint states, this object is outer-level
    # robot
    robot = moveit_commander.RobotCommander()

    # Instantiate a "PlanningSceneInterface". This provides a remote interface
    # for getting, setting, and updating the robot's internal understanding of
    # the surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate a "MoveGroupCommander". This object is an interface to a planning
    # group (group of joints). In this project, the robot that we will use is panda
    # therefore, set the group's name to be "panda_arm". If in some certain cases,
    # we are using a different robot, change this value to the name of your robot
    # arm planning group.

    # This interface can be used to plan and execute motions:
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Create a "DisplayTrajectory" ROS publisher which is used to visualise
    # trajectories in RVIZ:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    """ --------------------- Getting Basic Information -------------------------- """
    """
    One thing worth noting is that in my system, the version of python is 
    2.7, therefore, when print things, ignore parenthesis, otherwise, do 
    not forget to add them back
    """
    # Get the name of the reference frame for panda robot
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # Print the name of the end-effector link for this group, in this project
    # the end effector is supposed to be a 3D printer head although it has not
    # been completely designed yet
    end_effector = move_group.get_end_effector_link()
    print "============ End effector link: %s" % end_effector

    # Get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # For debugging purposes, print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()


    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = end_effector
    self.group_names = group_names


  def go_to_joint_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group


    """ ------------------------------- Planning to a Joint Goal ------------------------------ """
    # The Panda's zero configuration is at a singularity
    # <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>"
    # so the first thing to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = -pi/2
    joint_goal[3] = -1.69
    joint_goal[4] = 0
    joint_goal[5] = 2.43
    joint_goal[6] = 0.8

    # Call go command with some pre-set joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling 'stop()' ensures that there is no residual movement
    move_group.stop()

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  # Define a class named as move_group

  def go_to_pose_goal(self):
    move_group = self.move_group

    """ ------------------------------- Planning to a Pose Goal ------------------------------ """
    # Plan a motion for this group to a desired pose for the
    # end-effector - printing head
    # For any three-dimensional space, it requires a 6-degree os freedom
    # of specification, to uniquely identify a pose, namely, in this case, position
    # x, y and z as well as orientation including roll, pitch and yaw

    # In order to make it easier for computers to understanad, Quaternions for
    # rotation is applied using four numbers of vectors [x y z w],
    # http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0    # Angle of rotation
    # Vectors representing axis of rotation
    pose_goal.position.x = 0.28
    pose_goal.position.y = -0.7
    pose_goal.position.z = 1.0

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling 'stop()' to ensure that there is no residual movement. 
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    move_group.clear_pose_targets()

    # Similar to before, the following code is for testing:
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


# Most important Section
# Plan a Cartesian path by specifying a list of
# waypoints for the end-effector to go through

 #
  def plan_cartesian_path(self, scale=1):
    move_group = self.move_group

    # Cartesian Paths
    # Plan a Cartesian path directly by specifying a list of waypoints
    # for the end-effector to go through. If executing interactively in a
    # Python shell, set scale = 1.0.

    # Build A list called waypoints to contain the points of robotic arm
    waypoints = []

    # Now we define a list containing all the target positional points
    # the first target point is to move z axis first, and in this case,
    # we get the initial position of the robotic arm
    wpose = move_group.get_current_pose().pose
    # start from the simple geometry, say a rectangle
    # First move up (z)
    wpose.position.z -= scale * 0.5
    waypoints.append(copy.deepcopy(wpose))
    # after which we move the sideways in y direction
    wpose.position.y += scale * 0.4
    waypoints.append(copy.deepcopy(wpose))
    # Second move forward/backwards in z direction
    wpose.position.z += scale * 0.5
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.4   # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # As Cartesian path is to be interpolated at a resolution of 1 cm which is 
    # why we will specify 0.01 as the end_effector_translation in Cartesian
    # translation.

    # compute_cartesian_path is used to compute the joint trajectory automatically
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,       # waypoints to follow
                                       0.01,            # end_effector_step
                                       0.0)             # jump_threshold

    # Note: This is just the planning procedure rather than
    # asking move_group to actually move the robot yet
    
    return plan, fraction


  def display_trajectory(self, plan):

    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    # Displaying a Trajectory
    # Now, ask RViz to visualize a plan (aka trajectory). But the
    # group.plan() method does this automatically so this is not that useful
    # here (it just displays the same trajectory again):

    # A 'DisplayTrajectory' msg has two primary fields, trajectory_start and trajectory.
    # We populate the trajectory_start with our current robot state to copy over
    # any AttachedCollisionObjects and add our plan to the trajectory.
    # Display trajectories
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)

    display_trajectory_publisher.publish(display_trajectory);


  def execute_plan(self, plan):

      move_group = self.move_group
      move_group.execute(plan, wait=True)

    # Note that the robot's current joint state must be within some tolerance of the
    # first waypoint in the 'RobotTrajectory', and otherwise 'execute()' will fail



  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

    box_name = self.box_name
    scene = self.scene

    # If the Python node dies before publishing a collision object update message, the message
    # could get lost and the box will not appear. To ensure that the updates are made, we wait
    # until we see the changes reflected in the 'get_attached_objects()' and 'get_known_object_names()' lists.

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

"""-----------------------------------------------------------------------------------------------"""

def main():
  try:
    print "----------------------------------------------------------"
    print "----------------------------------------------------------"
    print "Welcome to the MoveIt MoveGroup Python Interface - Yulin's BEng Project"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print "----------------------------------------------------------"
    print "========= Press 'Enter' to begin the simulation by setting up the moveit_commander =========="
    raw_input()
    tutorial = roboticprintingcontrol()

    print "========= Press 'Enter'  to execute a movement using a joint state goal ========="
    raw_input()
    tutorial.go_to_joint_state()

    print "======== Press 'Enter'  to execute a movement using a pose goal ========"
    raw_input()
    tutorial.go_to_pose_goal()

    print "======== Press 'Enter' to plan and display a Cartesian path ========"
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path()

    print "======== Press 'Enter'  to display a saved trajectory (this will replay the Cartesian path)  ========"
    raw_input()
    tutorial.display_trajectory(cartesian_plan)

    print "======== Press 'Enter'  to execute a saved path ========"
    raw_input()
    tutorial.execute_plan(cartesian_plan)

    print "======== Done, Nail it!! ========"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
