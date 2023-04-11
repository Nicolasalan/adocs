# Como fazer o planejamento de caminho no ROS2
* O que significa Planejamento de Caminho
* Como fazer o planejamento de caminho no ROS2
* planejador Nav2
* controlador Nav2
* Nav2 bt-navigator
* Comportamentos de navegação, incluindo comportamentos de recuperação

## O que se entende por Planejamento de Caminho?
Planejamento de caminho significa planejar uma trajetória do ponto A ao ponto B, evitando os obstáculos ao longo do caminho.

### Iniciando o planejamento de caminho no ROS Nav2
Para iniciar o Path Planning, você iniciará vários nós. Além disso, você precisa ter alguns outros nós necessários para o planejamento do caminho:

**REQUISITOS**

Antes de iniciar o sistema de planejamento de caminho, você precisa executar os seguintes nodes:

* node nav2_map_server
* node nav2_amcl
**PARA LANÇAR O PLANEJAMENTO DO CAMINHO**

Você precisa lançar mais quatro nodes:

* node do planejador
* node controlador
* gerenciador de node de comportamentos de recuperação
* node do navegador da árvore de comportamento
* node nav2_lifecycle_manager para gerenciar os node
### 1. Lançamento do planner

O planejador ROS Nav2 é equivalente ao planejador global no ROS1. Sua tarefa é encontrar um caminho para o robô do ponto A ao ponto B.

Ele calcula o caminho evitando os obstáculos conhecidos incluídos no mapa. O cálculo do caminho é construído assim que o robô recebe um 2d_Goal_Pose. O planejador também tem acesso a uma representação ambiental global e dados do sensor armazenados em buffer (ou seja, mapa de custo global).

> Atualmente (Galactic), apenas um algoritmo do Planner disponível no ROS2, Nav2Fn_Planner.

Os campos que você precisa indicar no lançamento do nó são:

* O pacote nav2_planner fornece o controlador
* O executável é chamado planner_server
* Os parâmetros necessários são:
     O arquivo `yaml` que contém todos os parâmetros de configuração do node

```python
     package='nav2_planner',
     executable='planner_server',
     name='planner_server',
     output='screen',
     parameters=[nav2_yaml])
```

Aqui você pode encontrar um arquivo de configuração típico que você pode usar para seus robôs como base:
> planner_server.yaml
```yaml
planner_server:
     ros__parameters:
          expected_planner_frequency: 10.0
          use_sim_time: True
          planner_plugins: ["GridBased"]
          GridBased:
               plugin: "nav2_navfn_planner/NavfnPlanner"
               tolerance: 0.5
               use_astar: false
               allow_unknown: true
```
### 2. Lançamento do controlador

O controlador ROS Nav2 é equivalente ao Local Planner no ROS1. Sua principal tarefa é realizar o planejamento reativo do caminho desde a posição atual até alguns metros à frente (até o alcance dos sensores). Em seguida, ele constrói uma trajetória para evitar os obstáculos dinâmicos (que não aparecem no mapa, mas podem ser detectados com a ajuda dos dados do sensor), enquanto tenta seguir o plano global.

Também é responsável por gerar os comandos da roda, para fazer o robô seguir a trajetória.

Atualmente, existem apenas dois planejadores locais disponíveis no ROS2:

`dwb_controller`: geralmente usado com robôs de `acionamento diferencial`
`Controlador TEB`: geralmente usado com robôs semelhantes a `carros` (este ainda não está funcionando corretamente no Galactic)
Os campos que você precisa indicar no lançamento do nó são:

* O pacote nav2_controller fornece o controlador
* O executável é chamado controller_server
* Os parâmetros necessários são:
     O arquivo yaml que contém todos os parâmetros de configuração do nó
```python
     package='nav2_controller',
     executable='controller_server',
     output='screen',
     parameters=[controller_yaml])
```
Aqui você pode encontrar um arquivo de configuração típico que você pode usar para seus robôs como base:
> controller.yaml
```
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"] 
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    # Goal checker parameters
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
    # DWB parameters
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.26
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.26
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: True
      stateful: True
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0
```
### 3. Lançamento do coordenador de navegação

Você já viu o node que calcula o planejamento do caminho e o nó que gera os comandos da roda para seguir esse caminho.

Em seguida, está o nó que coordena o nó que chama o nó do planejador de caminho solicitando um caminho e, em seguida, chama o controlador para mover o robô ao longo dele. Esse nó é o bt_navigator.

Esses são os campos que você precisa indicar no lançamento do nó:

* O `bt_navigator` é fornecido pelo pacote nav2_bt_navigator
* O executável é chamado `bt_navigator`
* Os parâmetros necessários são: O arquivo `yaml` que contém todos os parâmetros de configuração do bt_navigator
```python
     package='nav2_bt_navigator',
     executable='bt_navigator',
     name='bt_navigator',
     output='screen',
     parameters=[bt_navigator_yaml])
```
> bt_navigator.yaml

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    default_nav_to_pose_bt_xml: "/home/user/ros2_ws/src/path_planner_server/config/behavior.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
```
> IMPORTANTE: o comportamento do bt_navigator é definido por um arquivo XML que contém a árvore de comportamento com esse comportamento. Essa árvore de comportamento inclui quando chamar o planejador de caminho, quando chamar o controlador e quando ativar um comportamento de recuperação.

> behavior.xml
```xml
<!--
  This Behavior Tree replans the global path periodically at 1 Hz, and has
  recovery actions. Obtained from the official Nav2 package
-->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      <SequenceStar name="RecoveryActions">
        <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
        <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
        <Spin spin_dist="1.57"/>
        <Wait wait_duration="5"/>
      </SequenceStar>
    </RecoveryNode>
  </BehaviorTree>
</root>
```
### 4. Lançamento do recovery_server

O que acontece se o robô não conseguir encontrar um caminho válido para o objetivo fornecido? E se o robô ficar preso no meio e não conseguir descobrir o que fazer?

Em tais situações, o robô usa comportamentos de recuperação. Esses movimentos simples e pré-definidos geralmente limpam a situação, chamados de árvores de comportamento.

O recovery_server é o nó responsável por executar os comportamentos de recuperação. Por exemplo, o bt_navigator chamará o recovery_server quando acreditar que o robô está travado.

Os campos que você precisa indicar no lançamento do nó são:

* O pacote `nav2_recoveries` fornece o controlador
* O executável é chamado de `recovery_server`
* Os parâmetros necessários são: o arquivo yaml que contém todos os parâmetros de configuração do recovery_server
```python
     package='nav2_recoveries',
     executable='recoveries_server',
     name='recoveries_server',
     parameters=[recovery_yaml],
     output='screen'),
```
Atualmente, existem três comportamentos de recuperação disponíveis no Nav2:

* spin - executa uma rotação no local por um determinado ângulo
* backup - realiza uma translação linear por uma determinada distância
* wait - traz o robô para um estado estacionário
No arquivo de configuração, você deve indicar quais desses comportamentos deseja carregar, para que estejam prontos para serem chamados pelo `bt_navigator`.

Aqui você pode encontrar um arquivo de configuração típico que você pode usar para seus robôs como base:
> recovery.yaml
```yaml
recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries/Spin"
    backup:
      plugin: "nav2_recoveries/BackUp"
    wait:
      plugin: "nav2_recoveries/Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_timeout: 0.1
    use_sim_time: true
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
```

**a)** Crie um novo pacote no espaço de trabalho ros2_ws chamado `path_planner_server`.

**b)** Crie o diretório de inicialização e configuração em `ros2_ws/src/path_planner_server`.

**c)** Escreva um arquivo de inicialização para iniciar o planejador de caminho com o nome `pathplanner.launch.py` onde todos os nós do planejador de caminho são lançados corretamente.

**d)** Crie um arquivo YAML para cada nó que o exija no diretório de configuração. Você pode usar os arquivos de configuração mostrados acima.

**e)** Compile este novo pacote de planejador de caminho. Lembre-se de modificar o `setup.py` apropriadamente. Lembre-se de fornecer a nova instalação.

**f)** Inicie o sistema de localização da lição anterior.

**g)** Abrir RVIZ:
```bash
rviz2
```
Além disso, carregue o arquivo localization_rviz_config. Em seguida, adicione a exibição Path e configure-a para o tópico /plan.
Configure a exibição do caminho com estilo de linha: outdoors e, em seguida, forneça uma largura de linha de 0,05.
Salve esta configuração do RVIZ dentro do diretório ros2_ws/src com o nome pathplanning_rviz_config.
**h)** Use RVIZ para fornecer a localização inicial do robô.

**i)** Inicie o planejador de caminho com o novo arquivo de inicialização que você criou.

Lembre-se de que você precisa incluir o lançamento de um `nav2_lifecycle_manager` que precisa incluir todos os nós do planejador de caminho. Este gerenciador de ciclo de vida deve ter um nome diferente daquele lançado na localização.

Envio de um objetivo de navegação através do RVIZ
Agora que você tem tudo funcionando, tente dar ao robô uma meta de navegação.

**j)** Clique no botão RVIZ `Navigation2_Goal` e depois clique em qualquer ponto do mapa para direcionar o seu robô.

Você deve ver uma linha verde indicando o caminho que o robô seguirá, desde sua localização atual até o objetivo. Você também deve ver como o robô se move em direção ao seu objetivo.

> pathplanner.launch.py
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery.yaml')

    
    return LaunchDescription([     
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),
            
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            parameters=[recovery_yaml],
            output='screen'),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'recoveries_server',
                                        'bt_navigator']}])
    ])
```
## Os parâmetros de planejamento de caminho no ROS2
### 1. Parâmetros para os planejadores

Você pode encontrar a documentação oficial para os parâmetros de planejamento aqui. [link](https://navigation.ros.org/configuration/packages/configuring-planner-server.html)
```python
expected_planner_frequency: 10.0
use_sim_time: True
planner_plugins: ["GridBased"]
GridBased:
     plugin: "nav2_navfn_planner/NavfnPlanner"
     tolerance: 0.5
     use_astar: false
     allow_unknown: true
```
Atualmente, existem vários planejadores disponíveis no ROS2. O padrão é o Nav2Fn_Planner, que você usará aqui. Para outras configurações do planner, consulte a documentação oficial aqui.

**1.1 Parâmetros do Servidor do Planejador**

expected_planner_frequency: (duplo, 20)

> A frequência na qual calcular o caminho

planner_plugins: ("vetor", baseado em grade)

> Lista de nomes de plugins mapeados para parâmetros e solicitações de processamento. Cada namespace de plugin definido nesta lista precisa ter um parâmetro de plugin definindo o tipo de plugin a ser carregado no namespace, da seguinte forma:

```python
     planner_plugins: ["GridBased"]
     GridBased:
          plugin: "nav2_navfn_planner/NavfnPlanner"
```
**1.2. O Nav2Fn_Planner**

O Nav2Fn é um plugin para o Nav2 Planner Server. Ele pode usar os algoritmos A* ou Dijkstra para encontrar o caminho entre dois pontos.

No modo Dijkstra `(use_astar = false)`, o algoritmo de pesquisa de Dijkstra garante encontrar o caminho mais curto sob qualquer condição.
No modo A* `(use_astar = true)`, não é garantido que o algoritmo de busca de A* encontre o caminho mais curto; no entanto, usa uma heurística para expandir o campo potencial em direção ao objetivo, tornando mais rápido encontrar um caminho possível.

**Todos os parâmetros do nav2Fn_planner**
* `GridBased.tolerance` (duplo, 0,5): Tolerância em metros entre a pose de meta e o final do caminho.
* `GridBased.use_astar` (bool, falso): Se deve usar A*. Se falso, use a expansão de Dijkstra.
* `GridBased.allow_unknown` (bool, verdadeiro): Se deve permitir o planejamento em um espaço desconhecido.

### 2. Parâmetros para os controladores
Você pode encontrar a documentação oficial de todos os [parâmetros](https://navigation.ros.org/configuration/packages/configuring-controller-server.html) dos controladores [aqui](https://navigation.ros.org/configuration/packages/configuring-controller-server.html).

**2.1. Parâmetros do servidor controlador**

```python
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"] 
    controller_plugins: ["FollowPath"]
```

* `controller_frequency` (duplo, 20,0 Hz): Frequência para executar o controlador

* `min_x_velocity_threshold` (duplo, 0,01 m/s): Velocidade X mínima a ser considerada pelo controlador. Qualquer coisa abaixo disso será considerado como 0,0 m/s

* `min_y_velocity_threshold` (duplo, 0,0 m/s): Velocidade Y mínima a ser considerada pelo controlador. Qualquer coisa abaixo disso será considerado como 0,0 m/s

* `min_theta_velocity_threshold` (duplo, 0,1 rad/s): Velocidade angular mínima a ser considerada pelo controlador. Qualquer coisa abaixo disso será considerado como 0,0 rad/s

* `failure tolerance` (duplo, 0,5 m): A duração máxima em segundos que o plug-in do controlador chamado pode falhar antes que a ação nav2_msgs::action::FollowPath falhe. Defini-lo com o valor especial de -1,0 o torna infinito, 0 para desabilitar e qualquer valor positivo para o tempo limite apropriado.

* `progress_checker_plugin` (string, 'progress_checker'): Nome mapeado para o plug-in do verificador de progresso para verificar o progresso feito pelo robô.

* `goal_checker_plugins` (string, ['general_goal_checker']): A lista de nomes mapeados para plug-ins do verificador de metas para verificar se a meta foi alcançada.

* `controller_plugins` (vetor, ['FollowPath']): Lista de nomes mapeados para plug-ins do controlador para processamento de solicitações e parâmetros.

**2.2. DWB_Controller**

Essa implementação do controlador reescreve e estende a funcionalidade do dwa_local_planner do ROS1; assim, ele é chamado logicamente de DWBLocalPlanner.

Você pode encontrar a documentação oficial completa desse controlador [aqui](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html).

```yaml
FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.26
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.26
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: True
      stateful: True
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0
```

Os parâmetros mais importantes são:

Limites de velocidade e aceleração para os diferentes eixos. Eles controlam a rapidez e a precisão dos movimentos do robô.
`xy_goal_tolerance` (duplo, 0,25 m)
> Quanto erro pode ser aceito ao atingir a posição de meta

`Os críticos` são plug-ins que verificam a conclusão de diferentes restrições ao seguir o caminho local. Para saber mais sobre os críticos, consulte a documentação oficial.

**2.3. Verificador de progresso**
Verifica se o robô está preso ou progrediu para a conclusão da meta.

```yaml
progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
```
* `required_movement_radius` (duplo, 0,5 m): A quantidade mínima que um robô deve mover para progredir até a meta (m).

* `motion_time_allowance` (duplo, 10,0 s): A quantidade máxima de tempo que um robô tem para mover o(s) raio(s) mínimo(s).

**2.4. Verificador de metas**
Verifica se o robô atingiu a pose de meta.

```yaml

general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
```
* `xy_goal_tolerance` (duplo, 0,25 m): Tolerância para atender aos critérios de conclusão de metas (m).

* `yaw_goal_tolerance` (duplo, 0,25 rad): Tolerância para atender aos critérios de conclusão da meta (rad).

### 3. Parâmetros para a recuperação

**3.1 Servidor de recuperação**
```yaml
recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries/Spin"
    backup:
      plugin: "nav2_recoveries/BackUp"
    wait:
      plugin: "nav2_recoveries/Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_timeout: 0.1
    use_sim_time: true
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
```

* `costmap_topic`: (string, local_costmap/costmap_raw): Tópico de mapa de custo bruto para verificação de colisão.

* `footprint_topic`: (string, “local_costmap/published_footprint”): Tópico para pegada no quadro costmap.

* `cycle_frequency`: (duplo, 10,0): Frequência para executar plugins de recuperação.

* `transform_timeout`: (duplo, 0,1): Tolerância de transformação TF.

* `global_frame`: (string, “odom”): Quadro de referência.

* `robot_base_frame`: (string, “base_link”): Estrutura base do robô.

* `recovery_plugins`: (vector[string], {“spin”, “back_up”, “wait”}): Lista de nomes de plug-ins a serem usados, também corresponde a nomes de servidores de ação.

**3.2 navegador BT**
De acordo com a definição da documentação oficial: O módulo BT Navigator (Behavior Tree Navigator) implementa a interface de tarefa NavigateToPose. É uma implementação de navegação baseada em árvore de comportamento que visa permitir flexibilidade na tarefa de navegação e fornecer uma maneira de especificar facilmente comportamentos complexos de robôs, incluindo recuperação.
```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    default_nav_to_pose_bt_xml: "/home/user/ros2_ws/src/path_planner/config/behavior.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
```

* `default_nav_to_pose_bt_xml`: (string): Indique o caminho completo para o arquivo que contém o comportamento a ser usado para navegar para uma pose

* `default_nav_through_poses_bt_xml`: (string): Indique o caminho completo para o arquivo que contém o comportamento a ser usado para navegar pelas poses

* `bt_loop_duration`: (duplo, 10.0): Duração (em milissegundos) para cada iteração da execução do BT.

* `robot_base_frame`: (string, “base_link”): Estrutura da base do robô

* `global_frame`: (string, “mapa”): Quadro de referência global.

* `odom_topic`: (string, “odom”): Tópico no qual a odometria é publicada.

* `plugin_lib_names`: (vetor, nenhum): Lista de bibliotecas compartilhadas de nós da árvore de comportamento.

## Enviando uma meta de navegação pela linha de comando
Existem duas maneiras de fazer isso - usando o servidor de ação ou o tópico. Ambos são muito parecidos e podem ser usados quando você precisar de algum feedback sobre o resultado.

### Com um servidor de ação
Uma vez que o sistema de navegação usa um servidor de ação chamado /navigate_to_pose para receber metas do RVIZ, você pode usar a linha de comando para chamar o servidor de ação e fornecer uma meta ao robô.

Use o seguinte comando:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "pose: {header: {frame_id: map}, pose: {position: {x: 1.52, y: 1.92, z: 0.0}, orientation:{x: 0.0, y: 0.0, z: 0, w: 1.0000000}}}"
```
Você precisa alterar a posição e os valores de orientação da mensagem para o local no mapa para onde deseja enviar o robô (lembre-se de que a orientação requer um quaternion).

Você pode usar o RVIZ para identificar os valores de pose do local.

### Com um tópico

Obtenha as coordenadas do ponto do RVIZ via Publish Point. Clique em Publish Point e depois em um local no mapa. As coordenadas desse mapa aparecerão no terminal RVIZ.

Use o tópico pub do ros2 para publicar essas coordenadas no tópico /goal_pose.

O robô deve começar a se mover em direção a esse ponto.
```bash
ros2 topic pub -1 /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 2.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

### Enviando uma meta de navegação de um programa

1. Dentro do pacote `pathplanner_server`, crie um novo arquivo Python chamado nav_to_pose_action_client.py.
2. Este arquivo conterá seu programa ROS2, portanto, certifique-se de colocá-lo no local correto e adicionar uma entrada no setup.py para iniciá-lo.
3. Seu código Python deve fazer duas coisas:
     Inscreva-se no tópico /clicked_point. Você usará esse tópico para fornecer uma meta ao robô. Os dados do tópico são um Ponto nas coordenadas (x, y, z) do mapa. Essas são as coordenadas para as quais você terá que enviar o robô.
     Escreva um action client que seja ativado quando um novo ponto for publicado no tópico anterior. Em seguida, use os valores (x, y, z) para criar uma meta de navegação e enviá-la ao servidor de ações de navegação.
4. Crie um arquivo de inicialização para iniciar seu programa.
5. Compilar e instalar.
6. Inicie o seu programa. Em seguida, vá para RVIZ e forneça um local para o seu programa usando o recurso Publish Point.

> nav_to_pose_action_client
```python
import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class NavToPoseActionClient(Node):

    def __init__(self):
        super().__init__('Nav_To_Pose_Action_Client')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.subscriber_ = self.create_subscription(PointStamped, 'clicked_point', self.callback, 1)
        self.subscriber_  # prevent unused variable warning

    def callback(self, msg):
        self.get_logger().info('Recieved Data:\n X : %f \n Y : %f \n Z : %f' % (msg.point.x, msg.point.y, msg.point.z))
        self.send_goal (msg.point.x, msg.point.y, 0.0)

    def send_goal(self, x ,y, theta):
        self.get_logger().info('sending goal to action server')
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        goal_pose.pose.pose.position.x = x
        goal_pose.pose.pose.position.y = y
        goal_pose.pose.pose.position.z = theta

        self.get_logger().info('waiting for action server')
        self._action_client.wait_for_server()
        self.get_logger().info('action server detected')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_pose,
            feedback_callback=self.feedback_callback)
        self.get_logger().info('goal sent')

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}' + str(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('FEEDBACK:' + str(feedback) )

def main(args=None):
    rclpy.init(args=args)

    action_client = NavToPoseActionClient()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```