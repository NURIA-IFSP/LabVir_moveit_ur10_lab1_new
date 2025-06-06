#include <ros/ros.h>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// tau = 1 rotation in radiants
const double tau = 2 * M_PI;


void openGripper(trajectory_msgs::JointTrajectory& posture)
{
    posture.joint_names.resize(1);
    /* joint name alterado para o gripper pg70 */
    posture.joint_names[0] = "pg70_finger_left_joint";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(1);
    /* Posicao em que o gripper está aberto*/
    posture.points[0].positions[0] = 0.04;
    posture.points[0].time_from_start = ros::Duration(0.5);
}


void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
    posture.joint_names.resize(1);
     /* joint name alterado para o gripper pg70 */
    posture.joint_names[0] = "pg70_finger_left_joint";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(1);
    /* Posicao em que o gripper está fechado*/
    posture.points[0].positions[0] = 0.015;
    posture.points[0].time_from_start = ros::Duration(0.5);


}

void ur10_pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Grasp pose
    grasps[0].grasp_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(3.141, 0.004, -1.571); // Orientação obtida do rviz, ajustada para o pick
    // orientation.setRPY(0, M_PI, 0);
    
    // Posição obtida do tf tf_echo durante a execução do rviz [-1.000, -0.007, 0.494]
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    // grasps[0].grasp_pose.pose.position.x = -1.002;
    // grasps[0].grasp_pose.pose.position.y = 0.136;
    // grasps[0].grasp_pose.pose.position.z = 0.309;

    // Teste - posição funcionando
    // grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = -1.002;
    grasps[0].grasp_pose.pose.position.y = 0.007;
    grasps[0].grasp_pose.pose.position.z = 0.494;
    
    // Pre-grasp approach
    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    // Direction is set as positive x axis
    grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Post-grasp retreat
    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    //Direction is set as positive z axis 
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.05;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    // we need to open the gripper. We will define a function for that
    openGripper(grasps[0].pre_grasp_posture);

    // When it grasps it needs to close the gripper
    closedGripper(grasps[0].grasp_posture);

    // Set support surface as table 1
    move_group.setSupportSurfaceName("table1");

    // Set current state as start state
    move_group.setStartStateToCurrentState();

    // Call pick to pick up the object using the grasps given
    move_group.pick("object", grasps);

}

void ur10_place(moveit::planning_interface::MoveGroupInterface& group)
{
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Definindo a pose do objeto a ser colocado
    place_location[0].place_pose.header.frame_id = "base_link";

    tf2::Quaternion orientation;
    // Rotaciona o objeto 90 graus em torno do eixo Z
    orientation.setRPY(0, 0, M_PI / 2);  // tau / 4
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    // Posição de destino desejada (atualize conforme sua necessidade)
    place_location[0].place_pose.pose.position.x = 0.0;
    place_location[0].place_pose.pose.position.y = 1.0;
    place_location[0].place_pose.pose.position.z = 0.30;

    // Pre-place approach (aproximação antes de soltar o objeto)
    place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
    place_location[0].pre_place_approach.direction.vector.z = -1.0;  // Direção: para baixo
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    // Post-place retreat (retirada após soltar o objeto)
    place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
    place_location[0].post_place_retreat.direction.vector.z = 1.0;  // Retira para frente
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    // Abrir garra após posicionar
    openGripper(place_location[0].post_place_posture);

    // Nome da superfície de suporte (deve coincidir com a collision object)
    group.setSupportSurfaceName("table2");

    // Executar ação de place
    group.place("object", place_location);
}

void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    /* Definição do cenário pick and place */
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // Add the first table
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "base_link";

    // Define primitive dimension, position of the table 1 - pick position
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.5;
    collision_objects[0].primitives[0].dimensions[1] = 0.5;
    collision_objects[0].primitives[0].dimensions[2] = 0.5;
    
    // pose of table 1
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = -1.0;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    // Add tabe 1 to the scene
    collision_objects[0].operation = collision_objects[0].ADD;


    // Add the second table
    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "base_link";

    // Define primitive dimension, position of the table 2
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.5;
    collision_objects[1].primitives[0].dimensions[1] = 0.5;
    collision_objects[1].primitives[0].dimensions[2] = 0.5;
    // pose of table 2
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = 1.0;
    collision_objects[1].primitive_poses[0].position.z = 0;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    // Add tabe 2 to the scene
    collision_objects[1].operation = collision_objects[1].ADD;

    // Add the object to be picked
    collision_objects[2].id = "object";
    collision_objects[2].header.frame_id = "base_link";

    // Define primitive dimension, position of the object
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
    collision_objects[2].primitives[0].dimensions.resize(2);
    collision_objects[2].primitives[0].dimensions[0] = 0.12;
    collision_objects[2].primitives[0].dimensions[1] = 0.03;
    
    // pose of object
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = -1.0;
    collision_objects[2].primitive_poses[0].position.y = 0;
    collision_objects[2].primitive_poses[0].position.z = 0.31;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    // Add tabe 2 to the object
    collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);

}


int main(int argc, char** argv)
{
    // Inicialização do ROS - novo node name
    ros::init(argc, argv, "ur10_pick_and_place_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // Initialize MoveIt! components
    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    group.setPlanningTime(45.0);

    // Put the object in the scene
    addCollisionObject(planning_scene_interface);

    // Wait for initialization
    ros::WallDuration(1.0).sleep();


    // Rotina principal de pick and place

    // Pick the object
    ur10_pick(group);

    ros::WallDuration(1.0).sleep();

    // Place the object
    ur10_place(group);

    ros::waitForShutdown();
    return 0;

}