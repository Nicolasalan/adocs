# O repositório `ros2_controllers`

`ros2_control` oferece aos desenvolvedores um grande conjunto de transmissores e controladores genéricos que podem ser usados com diferentes tipos de sensores e atuadores em diferentes modos de controle. Por exemplo, os transmissores incluídos podem ler e publicar estados como estados de juntas (posição, velocidade e esforço), dados de IMU e valores de força-torque. Além da leitura, os controladores incluídos permitem enviar comandos usando o controle de posição, mas também os valores de velocidade e esforço. Esses recursos facilitam o início rápido de um pipeline `ros2_control` sem precisar começar do zero.

## Controladores Disponíveis
No momento da redação deste curso, a lista de controladores incluídos que podem ser escolhidos é a seguinte:

**Controladores:**

* controladores de posição
* controladores de esforço
* controladores de velocidade
* controlador de comando avançado
* controlador de trajetória conjunta
* controlador de unidade diferencial

> Nota: Os controladores são mantidos no repositório ros2_controllers, separados do código do framework ros2_control.

Visite https://github.com/ros-controls/ros2_controllers para obter a lista mais recente de controladores incluídos.

## Os controladores de posição

Um position_controller recebe uma entrada de posição e envia uma saída para uma interface de posição. Lembre-se de que ele apenas transfere o comando de entrada não modificado como saída para os atuadores. Devido a essa característica, é útil quando o loop de controle é integrado em hardware (por exemplo, usando o loop PID interno nos servos) e ao executar um sistema robótico em simulação onde tudo o que você deseja fazer é encaminhar os comandos de posição que são executados com perfeição.

Para usá-lo, especifique um nome de sua escolha e o tipo `position_controllers/JointGroupPositionController` dentro do arquivo de configuração do controlador `.yaml`, assim:

```yaml
    forward_position_controller:
      type: position_controllers/JointGroupPositionController
```
Seu arquivo de descrição do robô deve incluir uma interface de posição correspondente:
```xml
<command_interface name="position">
```
#### Usando position_controllers
Acesse `/ros2_ws/src/ros2_control_course/unit5` e localize o pacote ROS chamado `rrbot_unit5`. Dentro da pasta urdf, abra o arquivo de descrição do robô chamado `rrbot.xacro` e insira esta implementação pré-fornecida das tags XML `ros2_control`:
```xml
  <ros2_control name="position_controllers_exercise" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>

    <joint name="joint1">
      <command_interface name="position">
        <param name="min">-1000</param>
        <param name="max">1000</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <joint name="joint2">
      <command_interface name="position">
        <param name="min">-1000</param>
        <param name="max">1000</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

  </ros2_control>

  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find rrbot_unit5)/config/position_controller_config.yaml</parameters>
    </plugin>
  </gazebo>
```
Crie um arquivo chamado `position_controller_config.yaml` no diretório `/ros2_ws/src/ros2_control_course/unit5/rrbot_unit5/config/` que conterá o seguinte código:
>  position_controller_config.yaml
```yaml
# Controller manager configuration
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz

    # Declare controller name and type
    forward_position_controller:
      type: position_controllers/JointGroupPositionController
        
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
        
# Controller properties and joints to use
forward_position_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
```
Salve esse arquivo `.yaml` e invoque colcon build em um shell para instalar o arquivo de configuração para que o sistema ROS2 possa localizá-lo.
```bash
cd ~/ros2_ws
colcon build --packages-select rrbot_unit5
```
Em seguida, execute o seguinte arquivo de inicialização que gerará o robô configurado em uma simulação e iniciará o nó `robot_state_publisher`:
```bash
ros2 launch spawn_robot spawn_rrbot.launch.py
```
Para testar o `position_controller`, carregue e inicie-o usando este comando:
```bash
ros2 control load_controller --set-state start forward_position_controller
```
> Output
```bash
Sucessfully loaded controller forward_position_controller into state active
```
Você também precisará carregar e iniciar um joint_state_broadcaster para ler todas as posições conjuntas e publicá-las no tópico `/joint_states`:
```bash
ros2 control load_controller --set-state start joint_state_broadcaster
```
> Output
```bash
Sucessfully loaded controller joint_state_broadcaster into state active
```
Vamos vê-lo funcionar.
```bash
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
 - 2.55
 - 2.55" -1
```
No Gazebo você deve mover as articulações do robô para a posição comandada:
Agora, desligue o controlador.
```bash
ros2 control set_controller_state forward_position_controller stop
```

#### Limpe antes de passar para o próximo exercício
Antes de passar para o próximo exercício, certifique-se de ter parado o nó robot_state_publisher iniciado anteriormente pressionando Ctrl + C no console onde você executou o arquivo `spawn_rrbot.launch.py`.

Exclua o antigo modelo de robô no Gazebo. Como o robô mostrado no momento no Gazebo implementa um plug-in do Gazebo que lê o arquivo de configuração .yaml, exclua-o para gerar um novo robô com um arquivo de configuração atualizado.

```bash
ros2 service call /delete_entity 'gazebo_msgs/DeleteEntity' '{name: "rrbot"}'
```
## Os controladores de esforço
Para usar o `effort_controller` fornecido, especifique um nome de sua escolha e o tipo de controlador `effort_controller/JointGroupEffortController` dentro do arquivo de configuração do controlador `.yaml`:

```yaml
    effort_controllers:
      type: effort_controllers/JointGroupEffortController
```
Seu arquivo de descrição do robô deve incluir uma interface de esforço correspondente:
```xml
<command_interface name="effort">
```
### Usando effort_controllers

Você deve usar as tags XML fornecidas abaixo e remover as tags XML adicionadas nos exercícios anteriores, elas não são mais necessárias.
```xml
  <ros2_control name="effort_controllers_exercise" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>

    <joint name="joint1">
      <command_interface name="effort">
        <param name="min">-1000</param>
        <param name="max">1000</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <joint name="joint2">
      <command_interface name="effort">
        <param name="min">-1000</param>
        <param name="max">1000</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

  </ros2_control>

  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find rrbot_unit5)/config/effort_controller_config.yaml</parameters>
    </plugin>
  </gazebo>
```
Aqui está um exemplo de configuração .yaml que implementa um `effort_controller` e um `joint_state_broadcaster`. Salve-o como um arquivo chamado `effort_controller_config.yaml` no diretório `~/ros2_ws/src/ros2_control_course/unit5/rrbot_unit5/config/`.

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz

    # Declare controller name and type
    effort_controllers:
      type: effort_controllers/JointGroupEffortController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

# Controller properties and joints to use
effort_controllers:
  ros__parameters:
    joints:
      - joint1
      - joint2
    command_interfaces:
      - effort
    state_interfaces:
      - position
      - velocity
      - effort
```

Salve esse arquivo `.yaml` e invoque colcon build em um shell para instalar o arquivo de configuração para que o sistema ROS2 possa localizá-lo.
```bash
cd ~/ros2_ws
colcon build --packages-select rrbot_unit5
```
Em seguida, execute o seguinte arquivo de inicialização que gerará o robô configurado na simulação e iniciará o nó `robot_state_publisher`:
```bash
ros2 launch spawn_robot spawn_rrbot.launch.py
```
Execute o seguinte comando de inicialização para carregar e iniciar o `flight_controller`:
```bash
ros2 control load_controller --set-state start effort_controllers
```
Você também precisará carregar e iniciar um joint_state_broadcaster para ler todas as posições conjuntas e publicá-las no tópico `/joint_states`:
```bash
ros2 control load_controller --set-state start joint_state_broadcaster
```
Para mover o robô usando o controlador de esforço:
```bash
ros2 topic pub /effort_controllers/commands  std_msgs/msg/Float64MultiArray "data:
- 10.0
- 4.0" -1
```
Agora vamos desligar o controlador.
```bash
ros2 control set_controller_state effort_controllers stop
ros2 control unload_controller effort_controllers
```
## Os controladores de velocidade

Os controladores de velocidade recebem uma entrada de velocidade e enviam uma saída de velocidade, apenas transferindo o sinal de entrada para o hardware ou simulação.

Para usá-lo, especifique um nome e o seguinte tipo de controlador `velocity_controllers/JointGroupVelocityController` dentro do arquivo de configuração do controlador `.yaml`.

```yaml
    # Declare controller name and type
    forward_velocity_controller:
      type: velocity_controllers/JointGroupVelocityController
```
Seu arquivo de descrição do robô deve incluir uma interface de velocidade correspondente:
```xml
<command_interface name="velocity">
```
## Usando controladores de velocidade
Vá em frente, abra seu editor de código e abra o arquivo de descrição do robô chamado `rrbot.xacro` do exercício anterior. Insira esta implementação pré-fornecida de tags XML `ros2_control`:
```xml
    <ros2_control name="velocity_controller_exercise" type="system">

    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>

      <joint name="joint1">

        <command_interface name="velocity">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>

        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="acceleration"/>
        
      </joint>

      <joint name="joint2">

        <command_interface name="velocity">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>

        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="acceleration"/>
        
      </joint>
    
    </ros2_control>

  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find rrbot_unit5)/config/velocity_controller_config.yaml</parameters>
    </plugin>
  </gazebo>
```

Crie um arquivo chamado `velocity_controller_config.yaml` no diretório `~/ros2_ws/src/ros2_control_course/unit5/rrbot_unit5/config/` que conterá o seguinte código:
> velocity_controller_config.yaml
```yaml
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz

    # Declare controller name and type
    forward_velocity_controller:
      type: velocity_controllers/JointGroupVelocityController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

# Controller properties and joints to use
forward_velocity_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
```
Salve esse arquivo `.yaml` e, como antes, crie o pacote para tornar o ROS2 ciente desse novo arquivo de configuração.
```bash
cd ~/ros2_ws
colcon build --packages-select rrbot_unit5
```
Em seguida, execute o seguinte arquivo de inicialização que gerará o robô configurado na simulação e iniciará o nó `robot_state_publisher`:

```bash
ros2 launch spawn_robot spawn_rrbot.launch.py
```
Para testar o `speed_controller`, carregue-o e inicie-o usando este comando:
```bash
ros2 control load_controller --set-state start forward_velocity_controller
```

Você também precisará carregar e iniciar um `joint_state_broadcaster` para ler todas as posições conjuntas e publicá-las no tópico `/joint_states`:
```bash
ros2 control load_controller --set-state start joint_state_broadcaster
```
Para mover o robô usando o controlador de velocidade:
```bash
ros2 topic pub /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray "data:
 - 0.5
 - 0.5" -1
```
## O controlador de comando de avanço

Um `forward_command_controller` cria um assinante para receber sinais de entrada e envia essa entrada inalterada para os atuadores. Como o nome revela, ele passa a entrada recebida para o robô. Para isso, o sinal de comando e a interface de hardware da junta devem ter o mesmo tipo, por exemplo, comandos de posição para uma junta controlada por posição e comandos de velocidade para uma junta controlada por velocidade. `forward_command_controllers` são particularmente úteis quando o loop de controle é integrado no hardware (por exemplo, usando o loop PID interno nos servos) e ao executar um sistema robótico em uma simulação onde tudo o que você deseja fazer é enviar comandos para o seu robô.

Para usá-lo, defina um nome de sua preferência e especifique o tipo `forward_command_controller/ForwardCommandController` dentro do arquivo de configuração do controlador `.yaml`, por exemplo:
```yaml
    forward_position_controller:
      type: forward_command_controller/ForwardCommandController
```
Um aspecto particular do `forward_command_controller` é que o nome da interface deve ser definido por um parâmetro. Portanto, o arquivo de configuração `.yaml` deve incluir o parâmetro `interface_name` conforme mostrado abaixo e o arquivo de descrição do robô também deve incluir uma interface com o mesmo valor de parâmetro. Faça o exercício abaixo para entender melhor como isso é feito.

### Usando um forward_command_controller
Você ainda deve ter o arquivo `rrbot.xacro` do exercício anterior aberto. Caso contrário, vá em frente e abra-o agora. Remova as tags XML `ros2_control` do exercício anterior. As tags XML para este exercício têm a seguinte aparência:
```xml
<ros2_control name="forward_command_controllers_exercise" type="system">

    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>

      <joint name="joint1">
          <command_interface name="position">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>
        <command_interface name="velocity">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>
        <command_interface name="acceleration">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="acceleration"/>
      </joint>
      <joint name="joint2">
          <command_interface name="position">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>
        <command_interface name="velocity">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>
        <command_interface name="acceleration">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="acceleration"/>
      </joint>
  </ros2_control>

  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <robot_sim_type>gazebo_ros2_control/GazeboSystem</robot_sim_type>
      <parameters>$(find rrbot_unit5)/config/forward_command_controller_config.yaml</parameters>
    </plugin>
  </gazebo>
```
Crie um arquivo chamado `forward_command_controller_config.yaml` no diretório `~/ros2_ws/src/ros2_control_course/unit5/rrbot_unit5/config/` que conterá o seguinte código:
> forward_command_controller_config.yaml

```yaml
# Controller manager configuration
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz

    # Declare controller name and type
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    forward_position_controller:
      type: forward_command_controller/ForwardCommandController

# Controller properties and joints to use
forward_position_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
    interface_name: position
```
Observe a última linha:

* ***interface_name: posição***

Essa linha é particularmente importante porque informa ao forward_command_controller para encaminhar um sinal de comando para um conjunto de juntas com uma interface de posição. O nome da interface pode ser o que você quiser, é claro, você precisa ter uma interface de hardware correspondente com esse nome.

Salve esse arquivo `.yaml` e invoque `colcon build` em um shell para instalar o arquivo de configuração para que o sistema ROS2 possa localizá-lo.
```bash
cd ~/ros2_ws
colcon build --packages-select rrbot_unit5\
```
Em seguida, execute o seguinte arquivo de inicialização que gerará o robô configurado na simulação e iniciará o nó `robot_state_publisher`:
```bash
ros2 launch spawn_robot spawn_rrbot.launch.py
```
Para testar o `forward_command_controller`, carregue e inicie-o usando este comando:
```bash
ros2 control load_controller --set-state start forward_position_controller
```
Também precisaremos carregar e iniciar um `joint_state_broadcaster` para ler todas as posições conjuntas e publicá-las no tópico `/joint_states`:
```bash
ros2 control load_controller --set-state start joint_state_broadcaster
```
Vamos vê-lo funcionar.
```bash
ros2 tópico pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "dados:
  - 1.14
  - 1,14" -1
```
O `forward_position_controller` mantém as articulações do robô em uma posição fixa.

Agora vamos desligar o controlador.
```bash
ros2 control set_controller_state forward_position_controller stop
ros2 control unload_controller forward_position_controller
```

## Os controladores de esforço

Para usar o esforço_controller fornecido, especifique um nome de sua escolha e o tipo de controlador `effort_controllers/JointGroupEffortController` dentro do arquivo de configuração do controlador `.yaml`:
```yaml
    effort_controllers:
      type: effort_controllers/JointGroupEffortController
```
Seu arquivo de descrição do robô deve incluir uma interface de esforço correspondente:
```xml
<command_interface name="effort">
```

## O controlador de trajetória conjunta
O `joint_trajectory_controller` é um tipo diferente de controlador que move um grupo de juntas de forma sincronizada seguindo pontos de rota ou trajetórias pré-definidas. Esses waypoints cuidadosamente calculados podem comandar várias juntas para seguir um caminho programado, respeitando restrições como posição, velocidade e aceleração. Essas características especiais são inerentemente adequadas para braços robóticos, e é por isso que o `joint_trajectory_controller` fornecido pelo `ros2_control` é amplamente usado por robôs controlados por posição para interagir com o MoveIt!.

Para usar um `joint_trajectory_controller`, especifique o seguinte tipo de controlador dentro do arquivo de configuração do controlador `.yaml`:

```yaml
    position_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
```
O arquivo de descrição do robô também deve incluir uma interface de acordo com o valor do parâmetro `command_interfaces` definido no arquivo de configuração `.yaml` descrito abaixo.

**Representações de trajetória e geração de trajetória**

Trajetórias são caminhos compostos de waypoints consecutivos que incluem os momentos no tempo em que esses waypoints precisam ser alcançados. O controlador de trajetória conjunta interpola entre essas posições para criar comandos de movimento suave que seguem esses pontos de referência o mais próximo possível das marcas de tempo especificadas. Para ter sucesso, as juntas devem atingir os waypoints dentro de uma certa tolerância de meta, caso contrário o controlador informará uma falha. Opcionalmente, as trajetórias também podem incluir perfis de velocidade e aceleração.

Para gerar trajetórias de robôs, pacotes externos de planejamento de movimento, como o MoveIt! pode ser usado.

#### Usando controladores de trajetória conjunta
Acesse `/ros2_ws/src/ros2_control_course/unit5` e localize o pacote ROS chamado `rrbot_unit5`. Dentro da pasta urdf, abra o arquivo de descrição do robô chamado rrbot.xacro e insira esta implementação pré-fornecida das tags XML `ros2_control`:
```xml
  <ros2_control name="trajectory_controllers_exercise" type="system">

    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>

      <joint name="joint1">
          <command_interface name="position">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>
        <command_interface name="velocity">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>
        <command_interface name="acceleration">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="acceleration"/>
      </joint>
      <joint name="joint2">
          <command_interface name="position">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>
        <command_interface name="velocity">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>
        <command_interface name="acceleration">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="acceleration"/>
      </joint>
  </ros2_control>

  <!-- Gazebo ros2_control plugin -->

    <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <robot_sim_type>gazebo_ros2_control/GazeboSystem</robot_sim_type>
      <parameters>$(find rrbot_unit5)/config/joint_trajectory_controller_config.yaml</parameters>
    </plugin>
  </gazebo>
```

Um arquivo de configuração do controlador `.yaml` para usá-lo pode ser:
> joint_trajectory_controller_config.yaml
```yaml
# Controller manager configuration
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz

    # Declare controller name and type
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    position_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController


# Controller properties and joints to use
position_trajectory_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2

    command_interfaces:
      - position

    state_interfaces:
      - position

    state_publish_rate: 200.0 # Defaults to 50
    action_monitor_rate: 20.0 # Defaults to 20

    allow_partial_joints_goal: false # Defaults to false
    open_loop_control: true
    allow_integration_in_goal_trajectories: true
    constraints:
      stopped_velocity_tolerance: 0.01 # Defaults to 0.01
      goal_time: 0.0 # Defaults to 0.0 (start immediately)
```
Salve o arquivo de configuração dentro do diretório `/ros2_ws/src/ros2_control_course/unit5/rrbot_unit5/config`.

#### O arquivo de configuração explicado
* `juntas`: Nomes de juntas para controlar.

* `command_interfaces`: Interfaces de comando fornecidas pela interface de hardware para todas as juntas. Valores: [posição | velocidade | aceleração] (múltiplos permitidos)

* `state_interfaces`: Interfaces de estado fornecidas pelo hardware para todas as juntas. Valores: posição (obrigatório) [velocidade, [aceleração]]. A interface de aceleração só pode ser usada em combinação com posição e velocidade. Padrão: 50,0

* `state_publish_rate`: Taxa de publicação do tópico “estado” do controlador. Padrão: 20,0

* `action_monitor_rate`: Taxa para monitorar alterações de status quando o controlador está executando a ação (`control_msgs::action::FollowJointTrajectory`).

* `allow_partial_joints_goal`: Permite metas conjuntas definindo a trajetória apenas para algumas articulações.

* `open_loop_control`: Use o controlador no modo de controle de malha aberta ignorando os estados fornecidos pela interface de hardware e usando os últimos comandos como estados na próxima etapa de controle. Isso é útil se os estados do hardware não estiverem seguindo os comandos, ou seja, um deslocamento entre eles (típico para manipuladores hidráulicos). Se este sinalizador estiver definido, o controlador tenta ler os valores das interfaces de comando na inicialização. Se eles tiverem valores numéricos reais, eles serão usados em vez de interfaces de estado. Portanto, é importante definir as interfaces de comando para NaN (`std::numeric_limits::quiet_NaN()`) ou valores de estado quando o hardware é iniciado.

* `restrições`: Valores padrão para tolerâncias se nenhum valor explícito for um estado na mensagem `JointTrajectory`.

* `constraints.stopped_velocity_tolerance`: Valor padrão para desvio de velocidade final. Padrão: 0,01

* `constraints.goal_time`: Tolerância máxima permitida para não atingir o final da trajetória em um tempo pré-definido. Padrão: 0,0 (não marcado)

* `constraints.< joint_name>.trajectory` : Desvio máximo permitido da trajetória de destino para uma determinada junta. Padrão: 0,0 (tolerância não aplicada)

* `constraints.< joint_name>.goal`: Desvio máximo permitido da meta (final da trajetória) para uma determinada junta. Padrão: 0,0 (tolerância não aplicada)

Salve o arquivo `joint_trajectory_controller_config.yaml` no diretório `/ros2_ws/src/ros2_control_course/unit5/rrbot_unit5/config/` e construa o pacote para torná-lo visível para o ROS2.

```bash
cd ~/ros2_ws
colcon build --packages-select rrbot_unit5
```
Execute o seguinte arquivo de inicialização que gerará o robô configurado na simulação e iniciará o nó `robot_state_publisher`:

```bash
ros2 launch spawn_robot spawn_rrbot.launch.py
```
Em seguida, carregue e inicie-o usando este comando:
```bash
ros2 control load_controller --set-state start position_trajectory_controller
```
Carregue e inicie um `joint_state_broadcaster` para ler todas as posições conjuntas e publicá-las no tópico `/joint_states`:
```bash
ros2 control load_controller --set-state start joint_state_broadcaster
```
Inicie um script de nó publicador que publicará a trajetória conjunta desejada para o robô.
```bash
ros2 launch joint_trajectory_publisher joint_trajectory_publisher.launch.py
```
## O diff_drive_controller
Como você controlaria um robô móvel? Você usaria um controlador de posição, velocidade ou esforço? Se você pensou em velocidade, sua mente está indo na direção certa - mas você deve levar em consideração todos os movimentos possíveis do robô. Da teoria básica do movimento do robô, sabemos que um corpo rígido irrestrito em um plano tem três graus de liberdade: dois movimentos de translação ao longo dos eixos x e y e um movimento rotativo ao redor do eixo z. Portanto, uma base móvel pode ser melhor controlada com o que é chamado de twist, que é uma forma de representar velocidades lineares e angulares combinadas. O `diff_drive_controller` funciona de forma a poder ler as mensagens de torção e convertê-las em um comando de velocidade da roda direita e esquerda para os motores. A vantagem aqui é que você só precisa de um controlador para as rodas e não um por roda.

Além disso, o `diff_drive_controller` também lê os estados de posição e velocidade das rodas, estima a posição e a velocidade do robô móvel e publica uma mensagem `nav_msgs/Odometry` através da rede ROS2.

Lembre-se de que o `diff_drive_controller` incluído não possui nenhuma funcionalidade integrada de controle de malha fechada. Ao executar em simulação, isso não é particularmente problemático, mas em condições da vida real, um algoritmo de controle, como um controlador PID, deve ser adicionado. Então, se você planeja usar o `diff_drive_controller` para operação de um robô real, você precisa implementar um controlador de acionamento diferencial personalizado com controle de malha fechada ou usar atuadores com controle de velocidade de malha fechada integrado.

#### Usando o diff_drive_controller
O controlador `diff_drive_controller` aciona as juntas das rodas através de uma interface de velocidade.

Acesse `/ros2_ws/src/ros2_control_course/unit5` e localize o pacote ROS chamado `diffbot_description`. Entre no robots e abra `diffbot.ros2_control.xacro` para adicionar o plug-in de hardware e definir os nomes das juntas e nomes da interface de comando:

```xml
    <ros2_control name="diff_drive" type="system">
    
      <hardware>
          <plugin>gazebo_ros2_control/GazeboSystem</plugin>
      </hardware>
    
      <joint name="left_wheel_joint">
        <command_interface name="velocity">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
    
      <joint name="right_wheel_joint">
        <command_interface name="velocity">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
    
    </ros2_control>
```
#### Adicionar o plug-in Gazebo ROS
Como você sabe dos exercícios anteriores, ao executar em simulação, você precisa dizer ao Gazebo como mover as articulações do robô simulado. Neste exemplo, esse plug-in é definido em um arquivo xacro separado.

Dentro do pacote diffbot_description, vá até robots e abra o arquivo `diffbot.gazebo.xacro`. Adicione o seguinte bloco de código onde indicado pelos comentários do código.
```xml
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find diffbot_description)/config/diff_drive_controller_config.yaml</parameters>
    </plugin>
  </gazebo>
```
Agora crie um arquivo chamado `diff_drive_controller_config.yaml` no pacote `diffbot_description`, diretório de configuração que conterá o seguinte código:
>  diff_drive_controller_config.yaml
```yaml
controller_manager:
  ros__parameters:
    update_rate: 10  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    diffbot_base_controller:
      type: diff_drive_controller/DiffDriveController

diffbot_base_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]

    wheel_separation: 0.10
    #wheels_per_side: 1  # actually 2, but both are controlled by 1 signal
    wheel_radius: 0.015

    wheel_separation_multiplier: 1.0
    left_wheel_radius_multiplier: 1.0
    right_wheel_radius_multiplier: 1.0

    publish_rate: 50.0
    odom_frame_id: odom
    base_frame_id: base_link
    pose_covariance_diagonal : [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]

    open_loop: true
    enable_odom_tf: true

    cmd_vel_timeout: 0.5
    #publish_limited_velocity: true
    use_stamped_vel: false
    #velocity_rolling_window_size: 10

    # Velocity and acceleration limits
    # Whenever a min_* is unspecified, default to -max_*
    linear.x.has_velocity_limits: true
    linear.x.has_acceleration_limits: true
    linear.x.has_jerk_limits: false
    linear.x.max_velocity: 1.0
    linear.x.min_velocity: -1.0
    linear.x.max_acceleration: 1.0
    linear.x.max_jerk: 0.0
    linear.x.min_jerk: 0.0

    angular.z.has_velocity_limits: true
    angular.z.has_acceleration_limits: true
    angular.z.has_jerk_limits: false
    angular.z.max_velocity: 1.0
    angular.z.min_velocity: -1.0
    angular.z.max_acceleration: 1.0
    angular.z.min_acceleration: -1.0
    angular.z.max_jerk: 0.0
    angular.z.min_jerk: 0.0
```
> Nota: certifique-se de ter excluído o robô do exercício Gazebo anterior antes de continuar.

Agora é hora de testá-lo. Antes de continuar, você deve criar o espaço de trabalho para instalar o novo arquivo de configuração.
```bash
cd ~/ros2_ws && colcon build
source ~/ros2_ws/install/setup.bash
```
Agora, execute o seguinte comando para iniciar o Gazebo e o ROS 2 Control:
```bash
ros2 launch spawn_robot spawn_diffbot.launch.py
```
Para gerar o `diff_drive_controller` de um arquivo de inicialização, use o seguinte comando:
```bash
cd ~/ros2_ws && colcon build && source ~/ros2_ws/install/setup.bash
ros2 launch diffbot_sim diffbot_controllers.launch.py
```
Este comando deve imprimir uma série de linhas indicando informações sobre os processos iniciados e finalizados.

Você pode verificar isso usando o gerenciador do controlador para ver quais controladores estão em execução:
```bash
ros2 control list_controllers
# ou 
ros2 topic list
```
Agora você pode enviar um comando para o Diff Drive Controller:
```bash
ros2 topic pub --rate 30 /diffbot_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
 x: 0.2
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 1.0" -1
```