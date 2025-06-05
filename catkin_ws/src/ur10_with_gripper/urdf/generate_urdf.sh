#!/bin/bash

# Navegar para o diretório do pacote (opcional, remova se não for necessário)
cd "$(dirname "$0")" || exit

# Definir caminho de saída e nome do arquivo URDF
OUTPUT_URDF="ur10_with_gripper.urdf"
XACRO_FILE="ur10_with_gripper.urdf.xacro"

# Verificar se o arquivo xacro existe
if [ ! -f "$XACRO_FILE" ]; then
  echo "Erro: Arquivo XACRO '$XACRO_FILE' não encontrado!"
  exit 1
fi

# Executar o comando xacro
rosrun xacro xacro -o "$OUTPUT_URDF" "$XACRO_FILE" \
  joint_limit_params:="$(rospack find ur_description)/config/ur10/joint_limits.yaml" \
  kinematics_params:="$(rospack find ur_description)/config/ur10/default_kinematics.yaml" \
  physical_params:="$(rospack find ur_description)/config/ur10/physical_parameters.yaml" \
  visual_params:="$(rospack find ur_description)/config/ur10/visual_parameters.yaml"

# Verificar se o URDF foi gerado com sucesso
if [ -f "$OUTPUT_URDF" ]; then
  echo "URDF gerado com sucesso: $OUTPUT_URDF"
else
  echo "Erro: Falha ao gerar o URDF!"
  exit 1
fi