
# UR5 Pick and Place com MoveIt (ROS Noetic)

Este programa controla um robô UR5 utilizando MoveIt no ROS para realizar uma tarefa de pick and place (pegar e colocar objetos).

## 📦 Dependências

- ROS Noetic
- MoveIt
- Pacote do UR5 configurado no MoveIt
- `moveit_commander`
- `geometry_msgs`
- `moveit_msgs`
- `rospy`

---

## 🚀 Funcionamento do Código

O script executa os seguintes passos:

1. Inicializa o MoveIt Commander e o ROS.
2. Define o ambiente de cena (adiciona uma mesa e um objeto de pick).
3. Move o braço para a posição de pré-grasp.
4. Executa o grasp (pegar).
5. Move o objeto para a posição alvo (place).
6. Solta o objeto.

---

## 🧠 Explicação dos Componentes

### Inicialização do MoveIt

```python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
```

### Inicializa nós, grupos e cena

```python
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur5_pick_and_place', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path',
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20)
```

---

## 🗺️ Adicionando Objetos na Cena

```python
rospy.sleep(2)
scene.remove_world_object("table")
scene.remove_world_object("object")

# Adiciona a mesa
table_pose = geometry_msgs.msg.PoseStamped()
table_pose.header.frame_id = "base_link"
table_pose.pose.orientation.w = 1.0
table_pose.pose.position.x = 0.5
table_pose.pose.position.y = 0
table_pose.pose.position.z = 0.2
scene.add_box("table", table_pose, size=(0.4, 0.8, 0.4))

# Adiciona o objeto
object_pose = geometry_msgs.msg.PoseStamped()
object_pose.header.frame_id = "base_link"
object_pose.pose.orientation.w = 1.0
object_pose.pose.position.x = 0.5
object_pose.pose.position.y = 0
object_pose.pose.position.z = 0.4
scene.add_box("object", object_pose, size=(0.04, 0.04, 0.04))
```

---

## 🤖 Função para abrir e fechar o gripper (simulado)

```python
def open_gripper():
    rospy.loginfo("Abrindo gripper (simulado)")

def close_gripper():
    rospy.loginfo("Fechando gripper (simulado)")
```

---

## 🏗️ Função de Pick (Pegar)

```python
def pick(group):
    rospy.loginfo("Movendo para posição de pré-grasp")
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.5
    pose_goal.position.y = 0
    pose_goal.position.z = 0.6

    group.set_pose_target(pose_goal)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    rospy.loginfo("Descendo para grasp")
    pose_goal.position.z = 0.45
    group.set_pose_target(pose_goal)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    close_gripper()

    rospy.loginfo("Subindo com o objeto")
    pose_goal.position.z = 0.6
    group.set_pose_target(pose_goal)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
```

---

## 🎯 Função de Place (Colocar)

```python
def place(group):
    rospy.loginfo("Movendo para local de place")
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.3
    pose_goal.position.y = -0.3
    pose_goal.position.z = 0.6

    group.set_pose_target(pose_goal)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    rospy.loginfo("Descendo objeto")
    pose_goal.position.z = 0.45
    group.set_pose_target(pose_goal)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    open_gripper()

    rospy.loginfo("Voltando para cima")
    pose_goal.position.z = 0.6
    group.set_pose_target(pose_goal)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
```

---

## 🔄 Main

```python
if __name__ == "__main__":
    try:
        rospy.loginfo("Iniciando UR5 Pick and Place")
        open_gripper()

        pick(group)
        place(group)

        rospy.loginfo("Operação concluída")

    except rospy.ROSInterruptException:
        pass
```

---

## ⚙️ Execução

1. Ative o MoveIt com o robô UR5:

```bash
roslaunch ur5_moveit_config demo.launch
```

2. Execute este script:

```bash
rosrun cobot_IK ur5_pick_and_place.py
```

---

## 🚩 Observações Importantes

- Este script não controla o gripper real, apenas simula a abertura e fechamento.
- Se quiser controlar um gripper real, adicione comandos específicos (ex.: via `joint_states` ou tópicos do gripper).
- O erro anterior (`AttributeError: 'Grasp' object has no attribute 'support_surface_name'`) foi resolvido retirando essa linha, pois não é compatível com a API Python do MoveIt.

---

## ✍️ Autor

Luiz Marangoni  
`Núcleo de Robótica e Inteligência Artificial - IFSP`