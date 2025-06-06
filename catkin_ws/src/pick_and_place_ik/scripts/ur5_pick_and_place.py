#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import Grasp, PlaceLocation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def open_gripper(posture):
    """Abre a garra."""
    posture.joint_names = ["finger_joint"]

    point = JointTrajectoryPoint()
    point.positions = [0.04]  # Valor de abertura (ajuste conforme sua garra)
    point.time_from_start = rospy.Duration(0.5)

    posture.points.append(point)


def close_gripper(posture):
    """Fecha a garra."""
    posture.joint_names = ["finger_joint"]

    point = JointTrajectoryPoint()
    point.positions = [0.00]  # Valor de fechamento (ajuste conforme sua garra)
    point.time_from_start = rospy.Duration(0.5)

    posture.points.append(point)


def pick(group):
    """Executa a operação de pick."""

    grasps = []

    grasp = Grasp()
    grasp.grasp_pose.header.frame_id = "base_link"

    # Definindo a posição do objeto
    grasp.grasp_pose.pose.position.x = 0.4
    grasp.grasp_pose.pose.position.y = 0
    grasp.grasp_pose.pose.position.z = 0.2

    # Definindo a orientação do gripper
    grasp.grasp_pose.pose.orientation.w = 1.0

    # Pré-abordagem
    grasp.pre_grasp_approach.direction.header.frame_id = "base_link"
    grasp.pre_grasp_approach.direction.vector.z = -1.0
    grasp.pre_grasp_approach.min_distance = 0.05
    grasp.pre_grasp_approach.desired_distance = 0.1

    # Pós-retirada
    grasp.post_grasp_retreat.direction.header.frame_id = "base_link"
    grasp.post_grasp_retreat.direction.vector.z = 1.0
    grasp.post_grasp_retreat.min_distance = 0.05
    grasp.post_grasp_retreat.desired_distance = 0.1

    # Definindo abertura da garra antes do grasp
    open_gripper(grasp.pre_grasp_posture)

    # Definindo fechamento da garra no grasp
    close_gripper(grasp.grasp_posture)

    grasps.append(grasp)

    rospy.loginfo("Iniciando operação de pick...")
    group.pick("object", grasps)


def place(group):
    """Executa a operação de place."""

    places = []

    place = PlaceLocation()
    place.place_pose.header.frame_id = "base_link"

    # Definindo posição para colocar o objeto
    place.place_pose.pose.position.x = 0.4
    place.place_pose.pose.position.y = -0.3
    place.place_pose.pose.position.z = 0.2

    # Definindo orientação
    place.place_pose.pose.orientation.w = 1.0

    # Pré-abordagem
    place.pre_place_approach.direction.header.frame_id = "base_link"
    place.pre_place_approach.direction.vector.z = -1.0
    place.pre_place_approach.min_distance = 0.05
    place.pre_place_approach.desired_distance = 0.1

    # Pós-retirada da garra
    place.post_place_retreat.direction.header.frame_id = "base_link"
    place.post_place_retreat.direction.vector.y = -1.0
    place.post_place_retreat.min_distance = 0.05
    place.post_place_retreat.desired_distance = 0.1

    # Abrindo a garra após soltar o objeto
    open_gripper(place.post_place_posture)

    places.append(place)

    rospy.loginfo("Iniciando operação de place...")
    group.place("object", places)




def main():
    moveit_commander.rospy_initialize(sys.argv)
    rospy.init_node('ur5_pick_and_place', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = MoveGroupCommander("manipulator")

    rospy.sleep(2)

    rospy.loginfo("Adicionando objeto e mesa na cena...")

    # Adicionando uma mesa
    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = "base_link"
    table_pose.pose.position.x = 0.5
    table_pose.pose.position.y = 0
    table_pose.pose.position.z = 0.1
    table_pose.pose.orientation.w = 1.0
    scene.add_box("table1", table_pose, (0.5, 1.5, 0.2))

    # Adicionando o objeto
    object_pose = geometry_msgs.msg.PoseStamped()
    object_pose.header.frame_id = "base_link"
    object_pose.pose.position.x = 0.4
    object_pose.pose.position.y = 0
    object_pose.pose.position.z = 0.2
    object_pose.pose.orientation.w = 1.0
    scene.add_box("object", object_pose, (0.04, 0.04, 0.04))

    rospy.sleep(2)

    pick(group)
    rospy.sleep(2)
    place(group)

    rospy.loginfo("Finalizado!")

    moveit_commander.rospy_shutdown()


if __name__ == '__main__':
    main()
