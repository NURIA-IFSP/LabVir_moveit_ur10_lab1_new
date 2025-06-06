#!/usr/bin/env python
"""
This script demonstrates how to use MoveIt! with a UR5 robot in ROS to perform inverse kinematics (IK) and motion planning, including gripper control.

Functions:
    close_gripper(gripper_group):
        Closes the robot's gripper by setting the joint value to a predefined position.

    main():
        Initializes MoveIt! and ROS nodes, sets up the robot and planning scene, defines a target pose for the end effector,
        plans a trajectory to the target pose, executes the planned motion if successful, and closes the gripper.

Usage:
    Run this script as a ROS node to move the UR5 manipulator to a specified pose and close the gripper using MoveIt!.
"""



import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import math

tau = 2 * math.pi


def close_gripper(gripper_group):
    joint_goal = gripper_group.get_current_joint_values()
    joint_goal[0] = 0.785  # ou ajuste conforme seu gripper
    gripper_group.go(joint_goal, wait=True)
    gripper_group.stop()


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_interface_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    move_group = moveit_commander.MoveGroupCommander("manipulator")
    gripper_group = moveit_commander.MoveGroupCommander("gripper")

    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20
    )

    planning_frame = move_group.get_planning_frame()
    rospy.loginfo("Reference frame: %s", planning_frame)

    eef_link = move_group.get_end_effector_link()
    rospy.loginfo("End effector link: %s", eef_link)

    # Define pose alvo
    target_pose = geometry_msgs.msg.Pose()
    quaternion = tf.transformations.quaternion_from_euler(-tau, tau / 4, tau / 4)

    target_pose.orientation.x = quaternion[0]
    target_pose.orientation.y = quaternion[1]
    target_pose.orientation.z = quaternion[2]
    target_pose.orientation.w = quaternion[3]

    target_pose.position.x = 0.5
    target_pose.position.y = -0.14
    target_pose.position.z = 0.3

    move_group.set_pose_target(target_pose)

    # Planejamento - desembrulhar a tupla
    result = move_group.plan()

    # Verificar se é tupla
    if isinstance(result, tuple):
        # Suporte para diferentes quantidades de itens na tupla
        if len(result) >= 2:
            success = result[0]
            plan = result[1]
        else:
            rospy.logerr("Resultado inesperado no planner: %s", str(result))
            return
    else:
        # Caso retorne apenas o plano (com versões antigas)
        plan = result
        success = True if plan and plan.joint_trajectory.points else False

    if success and plan and plan.joint_trajectory.points:
        rospy.loginfo("Planning SUCCEEDED")
        move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
    else:
        rospy.logwarn("Planning FAILED")

    rospy.sleep(1.0)
    close_gripper(gripper_group)

    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
