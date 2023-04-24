# Arquivos de Inicialização Avançados

* Como chamar outros lançamentos dentro do lançamento principal
* Como criar arquivos de inicialização mais complexos
* Passagem de parâmetro

## Use outros lançamentos dentro dos lançamentos
Este tópico não foi explicado anteriormente porque não era necessário. No entanto, é comum ter um lançamento principal que chama outros lançamentos em sistemas ROS2.

### **1. Crie um novo pacote** chamado `launch_tests_pkg`

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python launch_tests_pkg --dependencies rclpy launch_ros
```
### **2. Crie três novos arquivos** chamados `main.launch.py`, `move.launch.py` e `start_rviz.launch.py` dentro de uma pasta de inicialização dentro do pacote `launch_tests_pkg`. Crie também um diretório extra chamado `rviz_config` que conterá o arquivo de configuração Rviz.

```bash
cd ~/ros2_ws/src/launch_tests_pkg
mkdir launch
touch launch/main.launch.py

touch launch/move.launch.py
touch launch_tests_pkg/move_robot.py

touch launch/start_rviz.launch.py

mkdir rviz_config
```

Agora crie os seguintes lançamentos para fazer o seguinte:

* `move.launch.py`: Para iniciar o nó que publica no cmd_vel para mover o robô. Esse nó é fornecido pelo arquivo move_robot.py.
* `start_rviz.launch.py`: Inicia o RVIZ com o arquivo de configuração do RVIZ que você deseja
* `main.launch.py`: Inicia ambos os lançamentos com um único lançamento

#### move_robot.py
Este nó apenas cria um publicador para o tópico `/cmd_vel`. Em seguida, faz o robô girar a uma velocidade fixa. 

Coloque o seguinte código no arquivo move_robot.py dentro do diretório `~/ros2_ws/src/launch_tests_pkg/launch_tests_pkg`.

> move_robot.py

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MoveRobot(Node):

    def __init__(self):
        super().__init__('test')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd = Twist()
        self.turn()

    def turn(self):
        self.get_logger().info("TURNING....")
        self.cmd.linear.x = 0.5
        self.cmd.angular.z = 0.5
        self.publisher_.publish(self.cmd)

    def stop(self):
        self.get_logger().info("STOPPING....")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        self.get_logger().info("STOPPED")

    def __del__(self):
        self.stop()


def main(args=None):
    rclpy.init(args=args)

    try:
        move_robot_node = MoveRobot()
        rclpy.spin(move_robot_node)
        move_robot_node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
```
Observe o bloco **try/except** dentro da função principal:

```python
try:
    move_robot_node = MoveRobot()
    rclpy.spin(move_robot_node)
    move_robot_node.destroy_node()
    rclpy.shutdown()
except KeyboardInterrupt:
    pass
```
Essa estrutura pode ser utilizada para tratar possíveis erros (exceções) no seu código.

O programa executará o código dentro da instrução try. Se ocorrer um erro durante a execução deste código, uma exceção será lançada pelo Python. Podemos capturar essas exceções usando a instrução except.

Nesse caso, estamos capturando uma exceção específica, `KeyboardInterrupt`. Esta exceção será gerada sempre que pararmos nosso programa com Ctrl+C. Então, o que estamos fazendo aqui é capturar essa exceção e simplesmente ignorá-la (passar), para evitar um monte de mensagens de erro ao parar o programa.

#### move.launch.py
Este é o arquivo de inicialização que ativa o código `move_robot.py`. 

Coloque o seguinte código no arquivo move.launch.py dentro do diretório `~/ros2_ws/src/launch_tests_pkg/launch`.

> move.launch.py

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    move_robot_node = Node(
        package='launch_tests_pkg',
        executable='move_robot_exe',
        output='screen',
        name='move_robot_node',
        parameters=[{'use_sim_time': True}])

    # create and return launch description object
    return LaunchDescription(
        [
            move_robot_node
        ]
    )
```
#### start_rviz.launch.py
Inicie o Rviz com uma determinada configuração do Rviz especificada em um arquivo chamado `launch_part.rviz`.

Coloque o seguinte código no arquivo `start_rviz.launch.py` dentro do diretório `~/ros2_ws/src/launch_tests_pkg/launch`.

>  start_rviz.launch.py

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import launch


def generate_launch_description():

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(
        'launch_tests_pkg'), 'rviz_config', 'launch_part.rviz')

    # This is to publish messages inside Launch files.
    message_info = launch.actions.LogInfo(
        msg=str(rviz_config_dir))

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir])

    # create and return launch description object
    return LaunchDescription(
        [
            rviz_node,
            message_info
        ]
    )
```
**O que é `launch.actions.LogInfo()` ?**

No arquivo de inicialização anterior, estamos usando um recurso de ação de inicialização chamado LogInfo.

```python

message_info = launch.actions.LogInfo(
        msg=str(rviz_config_dir))

...


return LaunchDescription(
        [
            ...,
            message_info
        ]
    )
```

LogInfo, permite que você publique uma mensagem dentro do sistema de registro do ROS2 a partir de um arquivo de inicialização.

Nesse caso, a mensagem a ser impressa é o caminho completo para o arquivo de configuração do Rviz. Isso pode ser útil para verificar se estamos carregando o arquivo de configuração adequado.

Para imprimir uma mensagem durante o tempo de inicialização:

* você cria uma variável LogInfo, com argumento `(msg='a mensagem de texto que deseja imprimir')`.
* então você fornece essa variável para o `LaunchDescription`.

#### main.launch.py
Este arquivo de inicialização é responsável por iniciar os outros dois arquivos de inicialização. Para isso, usamos a ação IncludeLaunchDescription, bem como a função `PythonLaunchDescriptionSource`.

* `IncludeLaunchDescription` é fornecido por `launch.actions`. Importe-o com:
```python
from launch.actions import IncludeLaunchDescription
```
* `PythonLaunchDescriptionSource` é fornecido por `launch.launch_description_sources`. Importe-o com:

```python
from launch.launch_description_sources import PythonLaunchDescriptionSource
```
A maneira de usar esses dois elementos para iniciar dois arquivos de inicialização em um único arquivo de inicialização é com a seguinte estrutura:

```python
name_of_launch_object = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(MY_LAUNCH_FILE_PATH)
        )
    )

name_of_launch_object_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(MY_LAUNCH_FILE_PATH_2)
        )
    )
    
...


return LaunchDescription([
        name_of_launch_object,
        name_of_launch_object_2,

    ])
```
* A função `PythonLaunchDescriptionSource` deve conter o caminho completo para o arquivo de inicialização a ser iniciado.
* A ação `IncludeLaunchDescription` deve conter um objeto `PythonLaunchDescriptionSource` com o caminho para o arquivo de inicialização a ser iniciado.
Vamos aplicar isso ao nosso exemplo.

Coloque o seguinte código no arquivo main.launch.py dentro do diretório `~/ros2_ws/src/launch_tests_pkg/launch`.

> main.launch.py

```python
#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource




def generate_launch_description():

    pkg_box_bot_gazebo = get_package_share_directory('launch_tests_pkg')

    move_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_box_bot_gazebo, 'launch',
                         'move.launch.py'),
        )
    )

    start_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_box_bot_gazebo, 'launch', 'start_rviz.launch.py'),
        )
    )

    return LaunchDescription([
        move_robot,
        start_rviz,

    ])
```

#### setup.py

Finalmente, você precisa modificar o arquivo setup.py do pacote para:

* Adicione aos data_files o caminho para as configurações e inicializações que precisam ser instaladas durante a compilação
* Adicione um ponto de entrada para o nó `move_robot_exe`

Coloque o seguinte código no arquivo setup.py dentro do diretório `~/ros2_ws/src/launch_tests_pkg`.

> setup.py

```python 
from setuptools import setup
import os
from glob import glob

package_name = 'launch_tests_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz_config'), glob('rviz_config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_robot_exe = launch_tests_pkg.move_robot:main',
        ],
    },
)
```
Agora vamos compilar o pacote
```bash
cd ~/ros2_ws/
colcon build --symlink-install --packages-select launch_tests_pkg
source install/setup.bash
```
Agora inicie o `main.launch.py`.
```bash
source /home/simulations/ros2_sims_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch launch_tests_pkg main.launch.py
```
Leve em consideração que você ainda não possui este arquivo `launch_part.rviz`, portanto, inicialmente, você obterá uma janela RVIZ vazia.

Além disso, tenha em mente que a fonte a seguir é necessária para visualizar o `RobotModel`.

```bash
source /home/simulations/ros2_sims_ws/install/setup.bash
```
Agora você deve ver que:

* O robô começou a se mover.
* E a janela RVIZ aparece após 15-30 segundos. A primeira vez não terá nada, então adicione os elementos necessários e salve seu arquivo de configuração Rviz na pasta `rviz_config` com o nome `launch_part.rviz`. Em seguida, compile novamente e reinicie. Deverá ver o RVIZ aberto com a sua configuração guardada.


## Passagem de parâmetro

Outra coisa que você precisa saber é como passar argumentos entre inicializações e nós.

Aqui, crie alguns exemplos que mostrem o fluxo de como isso é feito porque não é trivial na primeira vez:

Crie todos os arquivos que você vai usar:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python launch_args_example_pkg --dependencies rclpy
cd ~/ros2_ws/src/launch_args_example_pkg
mkdir launch
touch launch/start_with_arguments.launch.py
touch launch/start_with_arguments_dummy.launch.py
chmod +x launch/start_with_arguments.launch.py
chmod +x launch/start_with_arguments_dummy.launch.py
touch launch_args_example_pkg/arguments_examples.py
```
Primeiro, crie um arquivo de inicialização simples (`start_with_arguments_dummy.launch.py`) que faz o seguinte:

* Imprima os valores de argumento que você passa pelo sistema de registro do ROS2.
* As entradas serão strings e floats.

#### start_with_arguments_dummy.launch.py

> start_with_arguments_dummy.launch.py

```python
import launch
# These log actions are: https://github.com/ros2/launch/blob/master/launch/launch/actions 
# Logging, for example: https://github.com/ros2/launch/blob/master/launch/launch/actions/log_info.py

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('extra_msg', default_value='hello world'),
        launch.actions.DeclareLaunchArgument('important_msg'),
        launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('extra_msg')),
        launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('important_msg')),
    ])
```

Este arquivo de inicialização faz duas coisas:

Ele declara como argumentos para este arquivo de inicialização dois argumentos. Um deles (msg) tem um valor padrão que não é obrigatório. O outro argumento é necessário porque não tem valor padrão.
Use o `launch.action.LogInfo` para registrar algumas mensagens.
Modifique o `setup.py` assim:

> setup.py

```python
from setuptools import setup
import os
from glob import glob

package_name = 'launch_args_example_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tgrip',
    maintainer_email='duckfrost@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```
Agora, compile e execute:
```bash
cd ~/ros2_ws
colcon build --packages-select launch_args_example_pkg
source install/setup.bash
ros2 launch launch_args_example_pkg start_with_arguments_dummy.launch.py extra_msg:="ROS2 is Great!" important_msg:="Execute Order 66"
```
Ótimo! Crie um nó simples que usa os argumentos fornecidos.

Este novo lançamento deve fazer o seguinte:

* Deverá receber como argumentos: duas strings que serão impressas alternadamente. Um float será usado para imprimir essa mensagem em um determinado período.
* Você deseja que o nó tenha apenas uma entrada de nome fornecida por meio de dois argumentos mutuamente exclusivos, mas eles não serão definidos como entrada por meio da linha de comando.

#### start_with_arguments.launch.py

> start_with_arguments.launch.py

```python
import launch
from launch_ros.actions import Node

# How to use Example:
# ros2 launch execution_and_callbacks_examples start_with_arguments.launch.py timer_period:=0.5


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('msg_A', default_value='Tick'),
        launch.actions.DeclareLaunchArgument('msg_B', default_value='Tack'),
        launch.actions.DeclareLaunchArgument('timer_period'),
        launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('msg_A')),
        launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('msg_B')),
        launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('timer_period')),
        # All the arguments have to be strings. Floats will give an error of NonItreable.
        Node(
            package='launch_args_example_pkg',
            executable='arguments_examples_demo',
            output='screen',
            emulate_tty=True,
            arguments=["-name_server", "TestArgumentsDemo", 
                        "-timer_period_message", launch.substitutions.LaunchConfiguration('msg_A'), 
                                                launch.substitutions.LaunchConfiguration('msg_B'),
                        "-timer_period", launch.substitutions.LaunchConfiguration('timer_period')
                    ]
            ),
    ])
```

#### arguments_examples.py

> arguments_examples.py

```python
# import the SetBool module from std_servs service interface
from pickle import TRUE
from std_srvs.srv import SetBool
import rclpy
from rclpy.node import Node
import time
import argparse

class DummyArgumetsExample(Node):

    def __init__(self, args):

        self.timer_flag = True
        
        super().__init__('dummy_arguments_example')

        # More info here: https://docs.python.org/3/library/argparse.html
        parser = argparse.ArgumentParser(
            description='Dummy Example for Arguments use')
        
        # All the arguments in this group will only be allowed to pass one of them as argument
        source = parser.add_mutually_exclusive_group(required=True)
        source.add_argument('-file', type=str, metavar='FILE_NAME',
                            help='Load entity xml from file')
        source.add_argument('-name_server', type=str, metavar='ENTITY_NAME',
                            help='Load entity name')

        # Warning: Setting the Nargs makes the variable now a list
        parser.add_argument('-timer_period_message',
                            type=str,
                            nargs=2,
                            help='Time the service will be waiting',
                            required=True)
        
        # Metvar will replace the default NAME_OF_ARGUMENT with the value shown in the command parser.print_help()

        parser.add_argument('-timer_period', 
                            type=float,
                            metavar='TIMEOUT', 
                            default=1.0,                           
                            help="Time period of the callback for timer")
        
        self.args = parser.parse_args(args[1:])

        # Check length of timer period message is 2
        assert len(self.args.timer_period_message) >= 2 , "You have to place two words"
        parser.print_help()

        self.timer = self.create_timer(float(self.args.timer_period), self.timer_callback)

    def timer_callback(self):
        self.print_dummy_msgs()

    def print_dummy_msgs(self):
        if self.timer_flag:
            self.get_logger().info(self.get_name()+"---"+self.args.timer_period_message[0])
            self.timer_flag = False
        else:
            self.get_logger().info(self.get_name()+"---"+self.args.timer_period_message[1])
            self.timer_flag = True

    def get_name(self):

        # Check which one was used and fill the var:
        final_name = "Nothing"
        try:            
            final_name = self.args.file
        except:
            pass

        try:            
            final_name = self.args.name_server
        except:
            pass  

        return final_name


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    print("args==="+str(args))
    # format the arguments given through ROS to use the arguments
    args_without_ros = rclpy.utilities.remove_ros_args(args)
    print("clean ROS args==="+str(args_without_ros))
    dummy_args_node = DummyArgumetsExample(args_without_ros)
    dummy_args_node.get_logger().info(" Started")
    # parser the program execution and wait for a request to kill the node (ctrl+c)
    rclpy.spin(dummy_args_node)
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Revisão de código
Esta linha é a encarregada de analisar os argumentos através do comando roslaunch e listar tudo em um array Python puro:

```python
args_without_ros = rclpy.utilities.remove_ros_args(args)
```
A matriz resultante ficará assim:

```bash
clean ROS args===['/home/user/ros2_ws/install/launch_args_example_pkg/lib/launch_args_example_pkg/arguments_examples_demo', '-name_server', 'TestArgumentsDemo', '-timer_period_message', 'Sith', 'Jedi', '-timer_period', '1.0']
```
Aqui você declara e analisa os argumentos:
```python
# More info here: https://docs.python.org/3/library/argparse.html
parser = argparse.ArgumentParser(
    description='Dummy Example for Arguments use')

# All the arguments in this group will only be allowed to pass one of them as argument
source = parser.add_mutually_exclusive_group(required=True)
source.add_argument('-file', type=str, metavar='FILE_NAME',
                    help='Load entity xml from file')
source.add_argument('-name_server', type=str, metavar='ENTITY_NAME',
                    help='Load entity name')

# Warning: Setting the Nargs makes the variable now a list
parser.add_argument('-timer_period_message',
                    type=str,
                    nargs=2,
                    help='Time the service will be waiting',
                    required=True)

# Metvar will replace the default NAME_OF_ARGUMENT with the value shown in the command parser.print_help()

parser.add_argument('-timer_period', 
                    type=float,
                    metavar='TIMEOUT', 
                    default=1.0,                           
                    help="Time period of the callback for timer")

self.args = parser.parse_args(args[1:])
```
Aqui você declara o analisador e algumas informações descritivas a serem impressas quando o help perguntar:
```python
parser = argparse.ArgumentParser(
    description='Dummy Example for Arguments use')
```
Crie um grupo para ter argumentos mutuamente exclusivos. Dessa forma, apenas UM pode ser usado como entrada.
```python
source = parser.add_mutually_exclusive_group(required=True)
```
Adicione o argumento ao grupo de origem criado:
```python

source.add_argument('-file', type=str, metavar='FILE_NAME',
                    help='Load entity xml from file')
source.add_argument('-name_server', type=str, metavar='ENTITY_NAME',
                    help='Load entity name')
```
Em seguida, adicione o argumento `timer_period_message`, que terá duas entradas. Isso é definido com os nargs. Se você fizer isso, acesse-os como uma matriz em Python. Você também está definindo conforme necessário.

```python
parser.add_argument('-timer_period_message',
                    type=str,
                    nargs=2,
                    help='Time the service will be waiting',
                    required=True)
```
Adicione outro argumento. Desta vez, use o metavar. Isso é para substituir na mensagem de ajuda o valor padrão que é o argumento em maiúsculas por um que você deseja. Não afeta o funcionamento interno.
```python
parser.add_argument('-timer_period', 
                    type=float,
                    metavar='TIMEOUT', 
                    default=1.0,                           
                    help="Time period of the callback for timer")
```
Salve todos os argumentos dentro de uma variável de classe chamada self.args, começando pelo segundo elemento, pois o primeiro está relacionado ao nome do script:
```python
self.args = parser.parse_args(args[1:])
```
> Output

```bash
clean ROS args===['/home/user/ros2_ws/install/launch_args_example_pkg/lib/launch_args_example_pkg/arguments_examples_demo', '-name_server', 'TestArgumentsDemo', '-timer_period_message', 'Sith', 'Jedi', '-timer_period', '1.0']
self.args===['-name_server', 'TestArgumentsDemo', '-timer_period_message', 'Sith', 'Jedi', '-timer_period', '1.0']
```
Modifique o `setup.py` para adicionar o novo ponto de entrada do nó.

> setup.py

```python
from setuptools import setup
import os
from glob import glob

package_name = 'launch_args_example_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tgrip',
    maintainer_email='duckfrost@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arguments_examples_demo = launch_args_example_pkg.arguments_examples:main',
        ],
    },
)
```
Agora, compile e execute:

```bash
cd ~/ros2_ws
colcon build --packages-select launch_args_example_pkg
source install/setup.bash
ros2 launch launch_args_example_pkg start_with_arguments.launch.py timer_period:=0.5
# Another example
ros2 launch launch_args_example_pkg start_with_arguments.launch.py msg_A:="Sith" msg_B:="Jedi" timer_period:=1.0
```

> Output

```bash
INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: Sith
[INFO] [launch.user]: Jedi
[INFO] [launch.user]: 1.0
[INFO] [arguments_examples_demo-1]: process started with pid [5447]
[arguments_examples_demo-1] args===None
[arguments_examples_demo-1] clean ROS args===['/home/user/ros2_ws/install/launch_args_example_pkg/lib/launch_args_example_pkg/arguments_examples_demo', '-name_server', 'TestArgumentsDemo', '-timer_period_message', 'Sith', 'Jedi', '-timer_period', '1.0']
[arguments_examples_demo-1] usage: arguments_examples_demo
[arguments_examples_demo-1]        [-h]
[arguments_examples_demo-1]        (-file FILE_NAME | -name_server ENTITY_NAME)
[arguments_examples_demo-1]        -timer_period_message
[arguments_examples_demo-1]        TIMER_PERIOD_MESSAGE
[arguments_examples_demo-1]        TIMER_PERIOD_MESSAGE
[arguments_examples_demo-1]        [-timer_period TIMEOUT]
[arguments_examples_demo-1]
[arguments_examples_demo-1] Dummy
[arguments_examples_demo-1] Example for
[arguments_examples_demo-1] Arguments
[arguments_examples_demo-1] use
[arguments_examples_demo-1]
[arguments_examples_demo-1] optional arguments:
[arguments_examples_demo-1]   -h, --help
[arguments_examples_demo-1]     show this
[arguments_examples_demo-1]     help
[arguments_examples_demo-1]     message and
[arguments_examples_demo-1]     exit
[arguments_examples_demo-1]   -file FILE_NAME
[arguments_examples_demo-1]     Load entity
[arguments_examples_demo-1]     xml from
[arguments_examples_demo-1]     file
[arguments_examples_demo-1]   -name_server ENTITY_NAME
[arguments_examples_demo-1]     Load entity
[arguments_examples_demo-1]     name
[arguments_examples_demo-1]   -timer_period_message TIMER_PERIOD_MESSAGE TIMER_PERIOD_MESSAGE
[arguments_examples_demo-1]     Time the
[arguments_examples_demo-1]     service
[arguments_examples_demo-1]     will be
[arguments_examples_demo-1]     waiting
[arguments_examples_demo-1]   -timer_period TIMEOUT
[arguments_examples_demo-1]     Time period
[arguments_examples_demo-1]     of the
[arguments_examples_demo-1]     callback
[arguments_examples_demo-1]     for timer
[arguments_examples_demo-1] [INFO] [1642516506.554290739] [dummy_arguments_example]:  Started
[arguments_examples_demo-1] [INFO] [1642516507.539556966] [dummy_arguments_example]: TestArgumentsDemo---Sith
[arguments_examples_demo-1] [INFO] [1642516508.539690047] [dummy_arguments_example]: TestArgumentsDemo---Jedi
```