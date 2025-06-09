
# Explicação do Código C++: Pick and Place com UR10 + PG70

Este documento fornece uma explicação detalhada do código C++ que implementa um sistema de pick and place utilizando o robô UR10 com a garra Schunk PG70, usando MoveIt no ROS.

---

## Visão Geral

O código realiza as seguintes etapas:
1. Inicialização do ROS e MoveIt.
2. Adição de objetos de colisão à cena (duas mesas e um objeto cilíndrico).
3. Execução do movimento de `pick`.
4. Execução do movimento de `place`.

---

## Estrutura do Código

### Includes Principais

```cpp
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
```

### Constantes

```cpp
const double tau = 2 * M_PI;
```
Tau representa 1 volta completa em radianos (2π).

---

## Controle do Gripper

### `openGripper` e `closedGripper`

Essas funções ajustam o valor da junta do gripper para abrir (0.04 m) ou fechar (0.015 m). A junta controlada é `"pg70_finger_left_joint"`.

```cpp
void openGripper(trajectory_msgs::JointTrajectory& posture)
{
    posture.joint_names.resize(1);
    posture.joint_names[0] = "pg70_finger_left_joint";
    posture.points.resize(1);
    posture.points[0].positions.resize(1);
    posture.points[0].positions[0] = 0.04;  // Posição aberta
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
    posture.joint_names.resize(1);
    posture.joint_names[0] = "pg70_finger_left_joint";
    posture.points.resize(1);
    posture.points[0].positions.resize(1);
    posture.points[0].positions[0] = 0.015;  // Posição fechada
    posture.points[0].time_from_start = ros::Duration(0.5);
}

```

---

## Movimento de Pick

### Função: `ur10_pick`

Define:
- A pose de grasp (`grasp_pose`) com posição e orientação obtidas no RViz.
- A abordagem pré-grasp e o recuo pós-grasp.
- Posturas do gripper antes e depois da pega.
- Usa `move_group.pick("object", grasps)` para executar a ação.

```cpp
void ur10_pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Configuração da pose de grasp
    grasps[0].grasp_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(3.141, 0.004, -1.571);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = -1.002;
    grasps[0].grasp_pose.pose.position.y = 0.007;
    grasps[0].grasp_pose.pose.position.z = 0.494;
    
    // Configuração da abordagem pré-grasp
    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Configuração do retorno pós-grasp
    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.05;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    openGripper(grasps[0].pre_grasp_posture);
    closedGripper(grasps[0].grasp_posture);

    move_group.setSupportSurfaceName("table1");
    move_group.setStartStateToCurrentState();
    move_group.pick("object", grasps);
}
```

---

## Movimento de Place

### Função: `ur10_place`

Define:
- A pose de colocação (`place_pose`) com orientação de 90º no eixo Z.
- Aproximação pré-place e recuo pós-place.
- Abre o gripper após o posicionamento.
- Executa com `group.place("object", place_location)`.

```cpp
void ur10_place(moveit::planning_interface::MoveGroupInterface& group)
{
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    place_location[0].place_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, M_PI / 2);
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
    place_location[0].place_pose.pose.position.x = 0.0;
    place_location[0].place_pose.pose.position.y = 1.0;
    place_location[0].place_pose.pose.position.z = 0.30;

    place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
    place_location[0].post_place_retreat.direction.vector.z = 1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    openGripper(place_location[0].post_place_posture);
    group.setSupportSurfaceName("table2");
    group.place("object", place_location);
}
```

---

## Adição de Objetos de Colisão

### Função: `addCollisionObject`

Adiciona 3 objetos:
- `table1`: Mesa no ponto (-1.0, 0, 0)
- `table2`: Mesa no ponto (0, 1.0, 0)
- `object`: Cilindro a ser manipulado, posicionado em (-1.0, 0, 0.31)

Utiliza a interface `PlanningSceneInterface` para aplicar os objetos ao ambiente de simulação do MoveIt.

```cpp
void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // Mesa 1 (pick)
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "base_link";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions = {0.5, 0.5, 0.5};
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position = {-1.0, 0, 0};
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    collision_objects[0].operation = collision_objects[0].ADD;

    // Mesa 2 (place)
    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "base_link";
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions = {0.5, 0.5, 0.5};
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position = {0, 1.0, 0};
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    collision_objects[1].operation = collision_objects[1].ADD;

    // Objeto a ser manipulado
    collision_objects[2].id = "object";
    collision_objects[2].header.frame_id = "base_link";
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
    collision_objects[2].primitives[0].dimensions = {0.12, 0.03}; // altura e raio
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position = {-1.0, 0, 0.31};
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

```
---

## Função Principal `main`

1. Inicializa ROS.
2. Inicializa `MoveGroupInterface` para o braço e para o gripper.
3. Chama `addCollisionObject()`.
4. Move o braço para a posição inicial `up`.
5. Executa `ur10_pick()` e depois `ur10_place()`.

```cpp
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur10_pick_and_place_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface group("manipulator");
    moveit::planning_interface::MoveGroupInterface gripper("gripper");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::WallDuration(1.0).sleep();
    group.setPlanningTime(45.0);

    addCollisionObject(planning_scene_interface);
    ros::WallDuration(1.0).sleep();

    // Move para posição inicial
    ROS_INFO("Movendo braco para posicao 'up'...");
    group.setNamedTarget("up");
    group.move();
    group.stop();
    ros::WallDuration(1.0).sleep();

    // Abre o gripper
    ROS_INFO("Abrindo gripper na posicao 'open'...");
    gripper.setNamedTarget("open");
    gripper.move();
    gripper.stop();
    ros::WallDuration(1.0).sleep();

    // Executa pick and place
    ur10_pick(group);
    ros::WallDuration(1.0).sleep();
    ROS_INFO("Posicionando o objeto 'place'..."); 
    ur10_place(group);

    ROS_INFO("Pick and place realizado ...");
    return 0;
}

```

---

## Considerações

- A manipulação depende da correta configuração da cena e nomes das juntas.
- O gripper PG70 é modelado como um único dedo com `mimic`, usando apenas a junta `"pg70_finger_left_joint"`.

---

## Requisitos

- ROS Noetic
- MoveIt
- UR10 + Schunk PG70 URDF/XACRO
- Objeto e mesas definidos como `collision_objects`

---

