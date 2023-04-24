
# Editor de estado do robô
* A função do Robot State Publisher
* Como usá-lo
* Editor de estado de robô vs. Editor de estado conjunto

## Gere um robô no Gazebo

Use o pacote gerado anteriormente nesta seção para criar um arquivo de inicialização que gera um robô dentro do Gazebo.
Para começar, precisamos de um novo arquivo de inicialização:
```bash
cd ~/ros2_ws/src/my_tf_ros2_course_pkg/launch
touch spawn_without_robot_state_publisher.launch.py
```

> spawn_without_robot_state_publisher.launch.py

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    urdf_file = 'unicycle.urdf'
    package_description = 'unicycle_robot_pkg'

    urdf = os.path.join(get_package_share_directory(
        package_description), 'urdf', urdf_file)

    xml = open(urdf, 'r').read()

    xml = xml.replace('"', '\\"')

    spawn_args = '{name: \"my_robot\", xml: \"' + xml + '\" }'
    
    spawn_robot =  ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity',
                 'gazebo_msgs/SpawnEntity', spawn_args],
            output='screen')

    return LaunchDescription([

     spawn_robot,
    ])
```

### Código explicado
Ao executar este arquivo de inicialização, ele chamará `generate_launch_description()` primeiro e processará a sequência de instruções em seu corpo de função.

As primeiras linhas são usadas para ler os dados de descrição do robô na variável `spawn_args`.

Em seguida, a ação ExecuteProcess chamada spawn_robot é definida com o argumento cmd correspondente. Isso significa que esse arquivo de inicialização executará uma chamada de serviço para o serviço `/spawn_entity` como se você fosse executar essa chamada de serviço na linha de comando.

Finalmente, temos que adicionar `spawn_robot` na lista `LaunchDescription`.

E é isso, este arquivo nos permitirá gerar um modelo de robô em simulação sem executar o nó `robot_state_publisher`. Abra-o e veja qual é o problema.

```bash
source /home/simulations/ros2_sims_ws/install/setup.bash
cd ~/ros2_ws && colcon build && source install/setup.bash
ros2 launch my_tf_ros2_course_pkg spawn_without_robot_state_publisher.launch.py
```
> Importante: obtenha primeiro o ros2_ws neste novo terminal:
```bash
source  ~/ros2_ws/install/setup.bash
rviz2
```

Adicione os seguintes elementos `RVIZ2` e sua configuração conforme explicado na unidade anterior:

* Configurar o Quadro Fixo
* Adicionar uma tela TF
* Adicione um modelo de robô. Observe que sem o nó `robot_state_publisher` em execução, a opção Topic não funcionará como Description Source, então você terá que usar File em seu lugar.

Então, como você pode ver, várias coisas estão acontecendo aqui:

* Você pode ver que o modelo do robô não tem malha adequada, é todo branco.
* Apenas o corpo principal é mostrado, faltando a cabeça.
* **RobotModel** mostra `Status:Erro`. Se você abrir esse menu para ver mais opções, encontrará várias mensagens de erro "Sem transformação de [..]".
A razão de todos esses erros é que você não tem as transformações para cada uma das partes do corpo do robô que estão sendo publicadas. Você pode criar um editor de transformação estática para cada uma das partes do corpo para resolver esse problema. Uma opção muito mais fácil é executar o editor de estado do robô, que é o que normalmente fazemos.

Observe, no entanto, que dois quadros tf estão realmente sendo publicados. A melhor maneira de vê-los é desmarcar a marca correspondente ao modelo do robô e usar a roda do mouse para aumentar o zoom.

Então, por que isso?

* Primeiro: já explicamos que os quadros das diferentes partes do corpo do robô não estão sendo publicados, porque você não tem nada que o faça. O responsável por fazer isso normalmente é o ROBOT STATE PUBLISHER.

* Segundo: Os dois quadros publicados são `unicycle_base_link` e odom. Esses quadros são publicados por um plug-in dentro do URDF do robô chamado `diferencial_drive`. É o responsável por receber o comando de velocidade para movimentar o robô e publicar a transformação no quadro de odometria.

Você pode ter uma visão melhor disso através das informações do tópico:

Verifique quem está publicando no tópico `/tf`:
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic info /tf --verbose
```
Você pode ver que o único publicador aqui de TFs é `diferencial_drive_controller`. E este controlador APENAS publica a transformação entre o `base_link` do robô e o quadro odom.

Se você mover o robô, os TFs se moverão de acordo sem nenhum problema.
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=unicycle_cmd_vel
```
Observe que você deve manter o foco no terminal clicando com o botão esquerdo nele.

> CONCLUSÃO: Você precisa do Robot State Publisher se quiser visualizar todas as partes do corpo do robô no Rviz.

## Publicador de Estado do Robô
O `Robot State Publisher` possui diversas funções que somente deve comentar agora:

Toma como ENTRADA o arquivo de descrição do modelo do robô URDF ou XACRO.
Ele publica esta descrição no tópico por padrão `/robot_description`, a partir do qual outras ferramentas como Gazebo ou Rviz irão ler.

Ele publica as transformações entre TODOS os links (partes do corpo) definidos no modelo do robô. Existem basicamente dois tipos de juntas que conectam links e, portanto, precisam de TFs transmitidos:

Juntas FIXED: Seus TFs são publicados diretamente, pois o `robot_state_publisher` não precisa de informações extras. O URDF é mais do que suficiente porque indica as transformações entre os links conectados.
Juntas MÓVEIS: Para publicar essas transformações, o `robot_state_publisher` precisa de uma informação adicional, o `joint_states`. Refere-se à informação sobre os ângulos ou a posição de todas as articulações que podem ser movidas, como rodas, cabeçotes, braços e pinças. Em robôs reais, isso é normalmente publicado pelos controladores do robô. No Gazebo, podemos iniciar o nó `joint_state_publisher` ou usar um plug-in do Gazebo que precisamos inserir no arquivo de descrição do modelo de robô URDF ou XACRO.
Em um arquivo de inicialização python ROS2, você pode adicionar um nó joint_state_publisher criando um objeto Node como este e, em seguida, incluir esse objeto na lista de argumentos que você passa para a função LaunchDescription:
```python
   joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

....

   return LaunchDescription([
        ....
        ....
        joint_state_publisher_node,
        robot_state_publisher_node,
        ......
        ......
    ])
```
Observe que, neste caso, o modelo do robô possui um `plug-in Joint State Publisher` dentro da descrição do modelo do robô. Portanto, não será necessário iniciar um Joint State Publisher Node separado por meio do arquivo de inicialização.

Crie um novo arquivo de inicialização que, neste caso, também iniciará o `robot_state_publisher`:
```bash
cd ~/ros2_ws/src/my_tf_ros2_course_pkg/launch
touch spawn.launch.py
```

> spawn.launch.py
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():


    # Position and orientation
    # [X, Y, Z]
    position = [0.0, 0.0, 1.0]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    robot_base_name = "unicycle_bot"


    entity_name = robot_base_name

    # Spawn ROBOT Set Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='cam_bot_spawn_entity',
        output='screen',
        emulate_tty=True,
        arguments=['-entity',
                   entity_name,
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', '/unicycle_bot_robot_description'
                   ]
    )

    ####### DATA INPUT ##########
    urdf_file = 'unicycle.urdf'
    #xacro_file = "box_bot.xacro"
    package_description = "unicycle_robot_pkg"

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)

    robot_desc = xacro.process_file(robot_desc_path)
    xml = robot_desc.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='unicycle_robot_state_publisher',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': xml}],
        remappings=[("/robot_description", "/unicycle_bot_robot_description")
        ],
        output="screen"
    )


    # create and return launch description object
    return LaunchDescription(
        [
            spawn_robot,
            robot_state_publisher_node
        ]
    )
```
Como você pode ver, a parte da desova do Gazebo é a mesma.

E o `Robot State Publisher` também é muito semelhante.

Você remapeia para usar o tópico `/unicycle_bot_robot_description` em vez do tópico `/robot_description`.
```bash
source /home/simulations/ros2_sims_ws/install/setup.bash
cd ~/ros2_ws && colcon build && source install/setup.bash
ros2 launch my_tf_ros2_course_pkg spawn.launch.py
```
Inicie o RVIZ2 e adicione o elemento novamente ou carregue um arquivo de configuração salvo.
```bash
rviz2
```
As texturas estranhas são um problema com o RVIZ relacionado à renderização de algumas texturas.

Se você mover o robô, verá que tudo funciona perfeitamente. E se você verificar o tópico TF agora, verá que há outra publicação de nó:

Verifique quem está publicando no tópico `/tf`:
```bash
ros2 topic info /tf --verbose
```

Agora você tem DOIS editores para TF:

* unicycle_robot_state_publisher
* diferencial_drive_controller

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=unicycle_cmd_vel
```
Você também pode dar uma olhada para ver o que o `Robot State Publisher` faz com o infocommand do nó `ros2`:
```bash
ros2 node info /unicycle_robot_state_publisher
```
Veja como tem como entrada `joint_states`
E como saída tem `/tf`, `/tf_static` (para as juntas fixas) e `/unicycle_bot_robot_description`.