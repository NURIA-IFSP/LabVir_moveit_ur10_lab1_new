
# Explicação do Código `ur5_ik_test.py`

Este script em Python controla o robô UR5 utilizando o MoveIt no ROS Noetic. O objetivo é mover o braço robótico para uma posição específica no espaço e fechar o gripper (garra) ao final.

---

## 🔧 Bibliotecas Importadas

```python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import math
```

- `rospy`: Biblioteca do ROS para nós em Python.
- `moveit_commander`: API Python do MoveIt para controle de braços robóticos e garras.
- `moveit_msgs.msg`: Mensagens ROS usadas para planejamento e visualização.
- `geometry_msgs.msg`: Para definição de posições e orientações.
- `tf`: Para cálculos de transformações, especialmente conversão entre Euler e Quaternions.
- `math`: Operações matemáticas básicas, como pi.

---

## 🔢 Constantes

```python
tau = 2 * math.pi
```
- `tau`: Valor de 2π, usado para cálculos de ângulos.

---

## ✋ Função para Fechar o Gripper

```python
def close_gripper(gripper_group):
    joint_goal = gripper_group.get_current_joint_values()
    joint_goal[0] = 0.785
    gripper_group.go(joint_goal, wait=True)
    gripper_group.stop()
```

- Obtém os valores atuais das juntas do gripper.
- Define a junta principal (`joint_goal[0]`) com o valor 0.785 (ângulo de fechamento).
- Executa o movimento e espera até a conclusão.
- `stop()` interrompe qualquer movimento residual, garantindo que pare exatamente no alvo.

---

## 🚀 Função Principal `main()`

### 🔸 Inicialização do ROS e MoveIt

```python
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_interface_tutorial', anonymous=True)
```

- Inicializa os sistemas do MoveIt e ROS.

### 🔸 Criação dos objetos principais

```python
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

move_group = moveit_commander.MoveGroupCommander("manipulator")
gripper_group = moveit_commander.MoveGroupCommander("gripper")
```

- `RobotCommander()`: Interface para informações do robô.
- `PlanningSceneInterface()`: Gerencia o ambiente e os objetos ao redor.
- `MoveGroupCommander("manipulator")`: Controla o grupo do braço.
- `MoveGroupCommander("gripper")`: Controla o grupo da garra.

### 🔸 Publicador de trajetórias para visualização

```python
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path',
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20
)
```

- Publica trajetórias planejadas que podem ser visualizadas no RViz.

### 🔸 Exibir informações do robô

```python
planning_frame = move_group.get_planning_frame()
rospy.loginfo("Reference frame: %s", planning_frame)

eef_link = move_group.get_end_effector_link()
rospy.loginfo("End effector link: %s", eef_link)
```

- Mostra o frame de referência e o link do efetuador final (ex.: `tool0`).

---

## 🎯 Definir a Pose Alvo

```python
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
```

- Cria um objeto `Pose` para definir posição e orientação desejadas.
- Utiliza `quaternion_from_euler` para converter ângulos Euler em quaternion.
- Define:
  - **Posição:** x=0.5, y=-0.14, z=0.3
  - **Orientação:** Definida pelo quaternion calculado.

---

## 🧠 Planejamento de Movimento

```python
result = move_group.plan()
```

- Gera um plano de movimento para alcançar a pose desejada.

### 🔍 Tratamento do retorno (compatível com diferentes versões)

```python
if isinstance(result, tuple):
    if len(result) >= 2:
        success = result[0]
        plan = result[1]
    else:
        rospy.logerr("Resultado inesperado no planner: %s", str(result))
        return
else:
    plan = result
    success = True if plan and plan.joint_trajectory.points else False
```

- Verifica se o retorno é uma tupla ou um único objeto.
- Extrai o plano e o status de sucesso de acordo com o formato retornado.

---

## 🚚 Execução do Movimento

```python
if success and plan and plan.joint_trajectory.points:
    rospy.loginfo("Planning SUCCEEDED")
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
else:
    rospy.logwarn("Planning FAILED")
```

- Se o planejamento for bem-sucedido:
  - Executa o movimento (`go()`).
  - Para movimentos residuais (`stop()`).
  - Limpa os alvos para evitar comandos pendentes (`clear_pose_targets()`).

---

## ✋ Fechar o Gripper

```python
rospy.sleep(1.0)
close_gripper(gripper_group)
```

- Aguarda 1 segundo.
- Fecha a garra.

---

## 🔚 Finalização do Script

```python
moveit_commander.roscpp_shutdown()
```

- Desliga as interfaces do MoveIt de forma limpa.

---

## 📜 Conclusão

Este script permite controlar o robô UR5 no ROS Noetic com MoveIt, realizando um planejamento de trajetória até uma pose específica no espaço, seguido de um comando para fechar o gripper.

---

## 📦 Dependências

- ROS Noetic
- MoveIt
- Pacote `tf` (transformações)
- Workspace com MoveIt configurado para o UR5
- Grupo de movimento chamado `"manipulator"` (braço)
- Grupo de movimento chamado `"gripper"` (garra)

---

## 🖥️ Visualização

É possível visualizar o plano e a execução no **RViz**, observando:
- O frame de referência (`world` ou outro definido).
- As trajetórias sendo desenhadas no ambiente 3D.

---
