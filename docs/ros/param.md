# Parâmetros do node
Parâmetros são valores que podem ser usados para configurar nós. No ROS2, os parâmetros são individuais para cada nó (sem roscore). Esta é uma diferença importante em relação ao ROS1. Portanto, quando o nó morre, os parâmetros associados também desaparecem. Os parâmetros podem ser carregados em um nó na inicialização ou durante a execução do nó.

## Crie um código de demonstração
Em seu `ros2_ws`, crie um novo pacote chamado `parameter_tests`:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python parameter_tests --dependencies rclpy
```
Dentro do diretório `parameter_tests/parameter_tests`, crie um novo script Python chamado `parameter_tests_node.py` e cole o código abaixo:

> parameter_tests_node.py

```python
import rclpy
import rclpy.node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist


class VelParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('param_vel_node')
        self.timer = self.create_timer(2, self.timer_callback)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.msg = Twist()
        param_descriptor = ParameterDescriptor(
            description='Sets the velocity (in m/s) of the robot.')
        self.declare_parameter('velocity', 0.0, param_descriptor)

    def timer_callback(self):
        my_param = self.get_parameter('velocity').value

        self.get_logger().info('Velocity parameter is: %f' % my_param)

        self.msg.linear.x = my_param
        self.publisher.publish(self.msg)


def main():
    rclpy.init()
    node = VelParam()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
```
Não se preocupe com o código agora. Você vai revisá-lo mais tarde.

Adicione um ponto de entrada ao arquivo `setup.py`:

> setup.py

```python
entry_points={
    'console_scripts': [
        'param_vel = parameter_tests.parameter_tests_node:main',
    ],
},
```

Por fim, crie o código:
```bash
cd ~/ros2_ws
colcon build --packages-select parameter_tests
source install/setup.bash
```
Agora você está pronto para executar o código!

```bash
ros2 run parameter_tests param_vel
```

> Output

```bash
[INFO] [1645178156.488937533] [param_vel_node]: Velocity parameter is: 0.000000
[INFO] [1645178158.454250656] [param_vel_node]: Velocity parameter is: 0.000000
[INFO] [1645178160.454262152] [param_vel_node]: Velocity parameter is: 0.000000
        
...
```

Parece que seu nó contém um parâmetro para configurar a velocidade, mas tente descobrir mais.

## Interagir com parâmetros das ferramentas de linha de comando
Primeiro, obtenha uma lista de todos os parâmetros atualmente disponíveis:

```bash
ros2 param list
```

> Output

```bash
/gazebo:  
  publish_rate
  qos_overrides./clock.subscription.depth
  qos_overrides./clock.subscription.durability
  qos_overrides./clock.subscription.history
  qos_overrides./clock.subscription.reliability
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  use_sim_time
/moving_service:
  use_sim_time
/param_vel_node:
  use_sim_time
  velocity
        
...
```
Como você pode ver, cada nó (`gazebo`, `Moving_service`, `param_vel_node`...) contém seus próprios parâmetros. Se você olhar mais de perto, encontrará o nó que acabou de lançar, `param_vel_node`, com um parâmetro chamado de velocidade.

Também é possível listar os parâmetros de um nó específico:

```bash
ros2 param list /param_vel_node
```

> Output

```bash
use_sim_time
  velocity
```

Cada nó possui o parâmetro use_sim_time. Ele é gerado automaticamente e usado para gerenciar o ROS Time: https://design.ros2.org/articles/clock_and_time.html.

Vamos agora obter mais alguns dados sobre este parâmetro de velocidade:

```bash
ros2 param describe /param_vel_node velocity
```

> Output

```bash
Parameter name: velocity
  Type: double
  Description: Sets the velocity (in m/s) of the robot.
  Constraints:
```

Você pode ver que o parâmetro de velocidade usa um tipo duplo, que parece configurar a velocidade do robô.

Você também pode obter o valor atual desse parâmetro de velocidade com o seguinte comando:
```bash
ros2 param get /param_vel_node velocity
```
Para obter acesso a um parâmetro, você deve primeiro especificar o nó ao qual ele pertence.

> Output

```bash
The double value is: 0.0
```
Então, e se você quiser alterar o valor desse parâmetro?

Você pode alterar o valor de um parâmetro em tempo de execução com o comando ros2 param set:

```bash
ros2 param set /param_vel_node velocity 0.1
```

> Output

```bash
Set parameter successful
```
No shell onde você está executando o programa, você verá a seguinte mensagem:

> Output

```bash
[INFO] [1645187457.780481759] [param_vel_node]: Velocity parameter is: 0.100000
```
Por fim, você também verá o robô se movendo - a uma velocidade de 0,1 m/s!

Se você quiser parar o robô novamente, tudo o que você precisa fazer é definir a velocidade de volta para 0.0

```bash
ros2 param set /param_vel_node velocity 0.0
```

> Observe que, se você usar o valor inteiro 0, receberá um erro como abaixo:

```bash
ros2 param set /param_vel_node velocity 0
```

> Output

```bash
Setting parameter failed: Wrong parameter type, expected 'Type.DOUBLE' got 'Type.INTEGER'
```

Às vezes é útil obter uma cópia dos parâmetros de um nó específico. Isso permite que você execute um nó com uma configuração específica quando necessário, por exemplo. Você pode fazer isso com o seguinte comando:

```bash
ros2 param dump /param_vel_node > param_vel_node.yaml
```
Isso irá gerar um arquivo chamado `param_vel_node.yaml` no caminho onde você executa o comando com o seguinte conteúdo:

> param_vel_node.yaml

```yaml
/param_vel_node:
  ros__parameters:
    use_sim_time: false
    velocity: 0.0
```
Como você pode ver, os parâmetros são armazenados no formato de arquivo YAML. Obviamente, também é possível carregar esse arquivo YAML em um nó em execução. Tente um exemplo rápido. Modifique o parâmetro de velocidade no arquivo YAML gerado:

> param_vel_node.yaml

```yaml
/param_vel_node:
  ros__parameters:
    use_sim_time: false
    velocity: 0.2
```
Agora, carregue o arquivo YAML com o seguinte comando:

```bash   
ros2 param load /param_vel_node param_vel_node.yaml
```

> Output

```bash   
Set parameter use_sim_time successful
Set parameter velocity successful
```
Você verá o robô começar a se mover a uma velocidade de 0,2 m/s!

Por fim, também é possível carregar parâmetros de um arquivo ao iniciar um nó.

Pare seu programa de parâmetros com Ctrl+C e reinicie-o com o seguinte comando:
```bash
ros2 run parameter_tests param_vel --ros-args --params-file /home/user/param_vel_node.yaml
```

### Resumo
* `ros2 param list` - Obtenha uma lista de todos os parâmetros

* `ros2 param get < node_name> < parameter_name>` - Obtenha o valor de um parâmetro específico

* `ros2 param set < node_name> < parameter_name> < parameter_value>` - Defina o valor de um parâmetro específico

* `ros2 param dump < node_name>` - Gera um arquivo YAML com os parâmetros atuais de um nó

* `ros2 param load < node_name> < parameter_file>` - Carrega os parâmetros se for um arquivo YAML em um nó em execução

### Examine o código
Agora que você interagiu com os parâmetros, observe o código que você criou em parte e tente entender o que ele está fazendo. Comece com o construtor da classe:

```python
class VelParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('param_vel_node')
        self.timer = self.create_timer(2, self.timer_callback)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.msg = Twist()
        param_descriptor = ParameterDescriptor(
            description='Sets the velocity (in m/s) of the robot.')
        self.declare_parameter('velocity', 0.0, param_descriptor)
```
A primeira coisa que você fará é criar um objeto timer. Este objeto timer é anexado a um `timer_callback` que será executado 1 vez a cada 2 segundos:
```python
self.timer = self.create_timer(2, self.timer_callback)
```
Em seguida, crie um objeto publicador para publicar mensagens no tópico `/cmd_vel` para mover o robô. Você também cria um objeto Twist:
```python
self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
```
Você define um ParameterDescriptor para fornecer alguns dados extras sobre ele.
```python
param_descriptor = ParameterDescriptor(
            description='Sets the velocity (in m/s) of the robot.')
```
Não é obrigatório criar um descritor para seus parâmetros. No entanto, é sempre útil fornecer dados sobre eles, especialmente para nós complexos com muitos parâmetros.

Por fim, declare seu parâmetro e forneça o seguinte:

* O nome do parâmetro: velocidade
* Um valor padrão para o parâmetro: 0,0.
* O descritor de parâmetro criado anteriormente.

```python
self.declare_parameter('velocity', 0.0, param_descriptor)
```
Em seguida, encontre o `timer_callback`:
```python
def timer_callback(self):
    my_param = self.get_parameter('velocity').value

    self.get_logger().info('Velocity parameter is: %f' % my_param)

    self.msg.linear.x = my_param
    self.publisher.publish(self.msg)
```
A linha mais importante é onde você obtém o valor desse parâmetro. Sem esta linha, você não poderia atualizar o valor do parâmetro em tempo de execução.
```python
my_param = self.get_parameter('velocity').value
```
Em seguida, defina o valor do parâmetro como a velocidade do robô e envie esta mensagem para o robô.

* Crie um novo nó com 2 parâmetros, get_laser_data e get_odom_data.
* Por padrão, ambos os parâmetros são definidos como Falso.
* Se definido como True, esses parâmetros começarão a exibir dados sobre o robô:

     * `get_laser_data`: exibe o valor do laser bem na frente do robô

     * `get_odom_data`: Exibe o valor do [X, Y] do robô

Você pode obter os dados necessários nos tópicos `scan` e `odom`.

## Carregar parâmetros no arquivo de inicialização
Também é possível definir parâmetros de um arquivo de inicialização. Nesse caso, os parâmetros serão definidos na inicialização, não no tempo de execução. Siga as instruções descritas abaixo para saber como.

Dentro do pacote `parameter_tests`, crie um novo diretório chamado launch. Adicione um novo arquivo de inicialização chamado `test_parameters.launch.py` com o seguinte conteúdo:

>  test_parameters.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='parameter_tests',
            executable='param_vel',
            name='param_vel_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'velocity': 0.2}
            ]
        )
    ])
```
Lembre-se de atualizar o arquivo `setup.py` para instalar o arquivo de inicialização corretamente.

> setup.py

```python
from setuptools import setup
import os
from glob import glob

package_name = 'parameter_tests'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
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
            'param_vel = parameter_tests.parameter_tests_node:main',
        ],
    },
)
```
Crie o código:
```bash
cd ~/ros2_ws
colcon build --packages-select parameter_tests
source install/setup.bash
```
E execute seu arquivo de inicialização!
```bash
ros2 launch parameter_tests test_parameters.launch.py
```

Você deve ver o robô começar a se mover a 0,2 m/s!

## Calbacks de Parâmetros
Como você pode ver, no ROS2, você pode interagir e modificar os parâmetros a qualquer momento. Sempre que um parâmetro de nó é atualizado, você pode notificar seu nó sobre essa alteração para que ele possa executar as ações necessárias, se necessário.

Atualize seu `parameter_tests_node.py` com o conteúdo abaixo:

> parameter_tests_node.py

```python
import rclpy
import rclpy.node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult


class VelParam(rclpy.node.Node):

    def __init__(self):
        super().__init__('param_vel_node')
        self.timer = self.create_timer(2, self.timer_callback)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.msg = Twist()
        param_descriptor = ParameterDescriptor(
            description='Sets the velocity (in m/s) of the robot.')
        self.declare_parameter('velocity', 0.0, param_descriptor)
        self.add_on_set_parameters_callback(self.parameter_callback)

    def timer_callback(self):
        self.my_param = self.get_parameter('velocity').value

        self.get_logger().info('Velocity parameter is: %f' % self.my_param)

        self.msg.linear.x = self.my_param
        self.publisher.publish(self.msg)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'velocity' and param.type_ == Parameter.Type.DOUBLE:
                self.my_param = param.value
                self.get_logger().info('Velocity parameter changed!')
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    node = VelParam()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
```
Observe 2 alterações importantes no código. Primeiro, você adicionou a linha abaixo:
```python
self.add_on_set_parameters_callback(self.parameter_callback)
```
Esta linha indica que, quando um novo parâmetro for definido para este nó, o `self.parameter_callback` deve ser acionado.

Finalmente, você também tem a definição desta função `parameter_callback()`:

```python
def parameter_callback(self, params):
    for param in params:
        if param.name == 'velocity' and param.type_ == Parameter.Type.DOUBLE:
            self.my_param = param.value
            self.get_logger().info('Velocity parameter changed!')
    return SetParametersResult(successful=True)
```
Aqui, verifique a velocidade do parâmetro e atualize seu valor. Em seguida, imprima uma mensagem de log dizendo que o parâmetro Velocity mudou!. Não há nada extravagante aqui. No entanto, lembre-se de que nesta função de retorno de chamada, você pode fazer o que quiser: armazenar o valor do novo parâmetro, alterar um comportamento em seu nó ou ignorar todas as alterações.

Agora teste seu novo código!

Primeiro, construa-o:
```bash
cd ~/ros2_ws
colcon build --packages-select parameter_tests
source install/setup.bash
```
E execute o nó!
```bash
ros2 run parameter_tests param_vel
```
Agora, sempre que você definir um novo valor para o parâmetro de velocidade, a função `parameter_callback` é acionada:

> Output

```bash
[INFO] [1645204804.180455048] [param_vel_node]: Velocity parameter is: 0.000000
[INFO] [1645204806.180370593] [param_vel_node]: Velocity parameter is: 0.000000
[INFO] [1645204806.376622758] [param_vel_node]: Velocity parameter changed!
[INFO] [1645204808.180674694] [param_vel_node]: Velocity parameter is: 0.100000
[INFO] [1645204810.180668051] [param_vel_node]: Velocity parameter is: 0.100000
[INFO] [1645204812.180699077] [param_vel_node]: Velocity parameter is: 0.100000
[INFO] [1645204812.228716863] [param_vel_node]: Velocity parameter changed!
[INFO] [1645204814.180731798] [param_vel_node]: Velocity parameter is: 0.000000
[INFO] [1645204816.180764604] [param_vel_node]: Velocity parameter is: 0.000000
```