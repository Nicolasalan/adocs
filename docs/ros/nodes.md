# Gerenciando Nós

* Como gerenciar nós mais complexos
* Executores
* Grupos de retorno de chamada

## Gerenciando programas complexos
Você tem trabalhado com programas relativamente fáceis de gerenciar. No entanto, alguns problemas podem aparecer conforme os programas que você cria se tornam mais complexos.

Comece criando um programa mais complexo.

### 1. Crie um novo pacote chamado `node_pkg`.
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python node_pkg --dependencies rclpy std_msgs sensor_msgs geometry_msgs nav_msgs
```

2. Crie um novo arquivo chamado node.py dentro da pasta node_pkg no pacote que você acabou de criar.
```python
import rclpy
from rclpy.node import Node
import time
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile

class ControlClass(Node):

    def __init__(self, seconds_sleeping=10):
        super().__init__('sub_node')
        self._seconds_sleeping = seconds_sleeping
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist()
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.laser_msg = LaserScan()
        self.odom_msg = Odometry()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def odom_callback(self, msg):
        self.get_logger().info("Odom CallBack")
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = self.euler_from_quaternion (orientation_list)

    def scan_callback(self, msg):
        self.get_logger().info("Scan CallBack")
        self.laser_msg = msg

    def get_front_laser(self):
        return self.laser_msg.ranges[360]

    def get_yaw(self):
        return self.yaw

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to Euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Below should be replaced when porting for ROS2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def stop_robot(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.vel_pub.publish(self.cmd)

    def move_straight(self):
        self.cmd.linear.x = 0.08
        self.cmd.angular.z = 0.0
        self.vel_pub.publish(self.cmd)
    
    def rotate(self):
        self.cmd.angular.z = -0.2
        self.cmd.linear.x = 0.0
        self.vel_pub.publish(self.cmd)
        
        self.get_logger().info("Rotating for "+str(self._seconds_sleeping)+" seconds")
        for i in range(self._seconds_sleeping):
            self.get_logger().info("SLEEPING=="+str(i)+" seconds")
            time.sleep(1)
        
        self.stop_robot()


    def timer_callback(self):
        self.get_logger().info("Timer CallBack")
        try:
            self.get_logger().warning(">>>>>>>>>>>>>>RANGES Value="+str(self.laser_msg.ranges[360]))
            if not self.laser_msg.ranges[360] < 0.5:
                self.get_logger().info("MOVE STRAIGHT")
                self.move_straight()
            else:
                self.get_logger().info("STOP ROTATE")
                self.stop_robot()
                self.rotate()
        except:
            pass
    

def main(args=None):
    rclpy.init(args=args)
    try:
        control_node = ControlClass()
        
        try:
            rclpy.spin(control_node)

        finally:
            control_node.destroy_node()

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Crie um arquivo de inicialização chamado `node_pkg_launch_file.launch.py` para iniciar o Node que você acabou de criar.

```bash
cd ~/ros2_ws/src/node_pkg
mkdir launch
cd launch   
cd ~/ros2_ws/src/node_pkg/launch
touch node_pkg_launch_file.launch.py
chmod +x node_pkg_launch_file.launch.py
```   
Adicione o seguinte código ao arquivo `node_pkg_launch_file.launch.py` que você criou.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='node_pkg',
            executable='node',
            output='screen',
            emulate_tty=True),
    ])
```
### 4. Modifique o setup.py para adicionar o arquivo de inicialização que você acabou de criar e os pontos de entrada para o executável do script `node.py`.

```python
from setuptools import setup
import os
from glob import glob

package_name = 'node_pkg'

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
    maintainer='somebody very awesome',
    maintainer_email='user@user.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node = node_pkg.node:main'
        ],
    },
)
```
### 5. Compile seu pacote.

```bash
cd ~/ros2_ws
colcon build --packages-select node_pkg
source ~/ros2_ws/install/setup.bash
```
Por fim, inicie o Node em seu shell.
```bash
ros2 launch node_pkg node_pkg_launch_file.launch.py
```

Então, algo estranho está acontecendo.

1. Parece que o retorno de chamada `/scan` foi substituído pelo retorno de chamada do Timer.
2. Parece que quando o robô está girando, nenhum outro Callback é executado.

> Output

```bash
[exercise31-1] [INFO] [1631462014.163838534] [sub_node]: Odom CallBack
[exercise31-1] [INFO] [1631462014.167065180] [sub_node]: Scan CallBack
[exercise31-1] [INFO] [1631462014.173721644] [sub_node]: Odom CallBack
[exercise31-1] [INFO] [1631462014.177145493] [sub_node]: Scan CallBack
    
# Here comes the timer and gets inside the STOP ROTATE
[exercise31-1] [INFO] [1631462014.183357178] [sub_node]: Timer CallBack
[exercise31-1] [WARN] [1631462014.183843557] [sub_node]: >>>>>>>>>>>>>>RANGES Value=0.43359941244125366
[exercise31-1] [INFO] [1631462014.184297364] [sub_node]: STOP ROTATE
[exercise31-1] [INFO] [1631462014.184905910] [sub_node]: Rotating for 10 seconds
[exercise31-1] [INFO] [1631462014.185361387] [sub_node]: SLEEPING==0 seconds
[exercise31-1] [INFO] [1631462015.187221747] [sub_node]: SLEEPING==1 seconds
[exercise31-1] [INFO] [1631462016.188986502] [sub_node]: SLEEPING==2 seconds
[exercise31-1] [INFO] [1631462017.190589988] [sub_node]: SLEEPING==3 seconds
[exercise31-1] [INFO] [1631462018.191754193] [sub_node]: SLEEPING==4 seconds
[exercise31-1] [INFO] [1631462019.192855738] [sub_node]: SLEEPING==5 seconds
[exercise31-1] [INFO] [1631462020.194047276] [sub_node]: SLEEPING==6 seconds
[exercise31-1] [INFO] [1631462021.195551992] [sub_node]: SLEEPING==7 seconds
[exercise31-1] [INFO] [1631462022.197470055] [sub_node]: SLEEPING==8 seconds
[exercise31-1] [INFO] [1631462023.199393248] [sub_node]: SLEEPING==9 seconds

# When it finishes, it executes again, because of stack priorities
# The sensor value is the same because the scan Callback has not been called yet.
[exercise31-1] [INFO] [1631462024.201929184] [sub_node]: Timer CallBack
[exercise31-1] [WARN] [1631462024.202541739] [sub_node]: >>>>>>>>>>>>>>RANGES Value=0.43359941244125366
[exercise31-1] [INFO] [1631462024.203118334] [sub_node]: STOP ROTATE
[exercise31-1] [INFO] [1631462024.203789991] [sub_node]: Rotating for 10 seconds
[exercise31-1] [INFO] [1631462024.204353594] [sub_node]: SLEEPING==0 seconds
[exercise31-1] [INFO] [1631462025.206108691] [sub_node]: SLEEPING==1 seconds
[exercise31-1] [INFO] [1631462026.207503346] [sub_node]: SLEEPING==2 seconds
[exercise31-1] [INFO] [1631462027.209666668] [sub_node]: SLEEPING==3 seconds
[exercise31-1] [INFO] [1631462028.211459854] [sub_node]: SLEEPING==4 seconds
[exercise31-1] [INFO] [1631462029.213224914] [sub_node]: SLEEPING==5 seconds
[exercise31-1] [INFO] [1631462030.215086652] [sub_node]: SLEEPING==6 seconds
[exercise31-1] [INFO] [1631462031.215881326] [sub_node]: SLEEPING==7 seconds
[exercise31-1] [INFO] [1631462032.217448376] [sub_node]: SLEEPING==8 seconds
[exercise31-1] [INFO] [1631462033.218958477] [sub_node]: SLEEPING==9 seconds

# And now that the scan finally was called, and the sensor value updated
# The Timer Callback can now move straight.
[exercise31-1] [INFO] [1631462034.222230681] [sub_node]: Odom CallBack
[exercise31-1] [INFO] [1631462034.224277957] [sub_node]: Scan CallBack
[exercise31-1] [INFO] [1631462034.226654888] [sub_node]: Timer CallBack
[exercise31-1] [WARN] [1631462034.228090777] [sub_node]: >>>>>>>>>>>>>>RANGES Value=13.314866065979004
[exercise31-1] [INFO] [1631462034.229608321] [sub_node]: MOVE STRAIGHT
[exercise31-1] [INFO] [1631462034.232116453] [sub_node]: Odom CallBack
[exercise31-1] [INFO] [1631462034.234230636] [sub_node]: Scan CallBack
```

A razão pela qual o timer Callback só é chamado duas vezes quando há uma inversão de prioridade provavelmente está relacionada ao timer relacionado à criação de um timer que possui um comportamento exclusivo na pilha de rotação do Callback.

Como você pode ver, há um problema real aqui:

Você precisa que os dados do sensor sejam atualizados. Portanto, você não pode permitir que timer_callback interrompa os retornos de chamada do sensor. Se fosse um carro, você não poderia esperar que o sistema de controle terminasse de mover o carro para obter uma nova leitura do sensor.

Por outro lado, o sistema de controle deve ser muito regular porque você não pode esperar que um retorno de chamada de processamento do sensor termine para agir. Em sistemas reais, se os dados do sensor forem muito antigos, o sistema é PARADO por `MOTIVOS DE SEGURANÇA`.

Então o que você pode fazer? Você precisa de vários threads para evitar essa colisão entre Callbacks, e cada thread pode se comportar de maneira diferente, dependendo do tipo de sistema.

Para parar o movimento do robô a partir do shell, use o seguinte comando:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
"
```

Para redefinir a posição inicial do robô, use o seguinte comando:
```bash
ros2 service call /reset_world std_srvs/srv/Empty "{}"
```
Você também pode reiniciar a simulação pressionando o último ícone na janela de simulação:

## Executores e Grupos de Callback
Na seção anterior, o `thread` principal bloqueou a chamada de função para move_straight().

Dois componentes controlam a execução de Callbacks: executores e grupos de Callback.

Os executores são responsáveis pela execução real dos callbacks.

Grupos de retorno de chamada são usados para impor regras de simultaneidade para retornos de chamada.

### Executores
Um executor controla o modelo de threading usado para processar Callbacks. Callbacks são unidades de trabalho como Callbacks de assinatura, Timer Callbacks, Service Callbacks e respostas recebidas do cliente - um executor controla onde Thread Callbacks são executados.

Você encontrará dois tipos de executores:

* `MultiThreadedExecutor`: Executa callbacks em um pool de threads.

* `SingleThreadedExecutor`: Executa callbacks no thread que chama `Executor.spin()`.

Grupos de retorno de chamada
Um grupo de retorno de chamada controla quando os retornos de chamada podem ser executados.

Esta classe não deve ser instanciada. Em vez disso, as classes devem estendê-lo e implementar `can_execute()`, `Beginning_Execution()` e `Ending_Execution()`.

Você pode encontrar diferentes tipos de grupos de retorno de chamada:

* `ReentrantCallbackGroup`: Permite que Callbacks sejam executados em paralelo sem restrições.

* `MutuallyExclusiveCallbackGroup`: permite que apenas um retorno de chamada seja executado por vez.

Agora, faça algumas modificações no programa anterior, adicionando um executor e usando Grupos de Callback.

### 1. Crie um novo arquivo chamado node1.py com o código mostrado logo abaixo.

Você afirma explicitamente que está usando um único thread e que todos os retornos de chamada estão no mesmo thread.

> node1.py

```python
import rclpy
from rclpy.node import Node
import time
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import ReliabilityPolicy, QoSProfile

class SubscriberClass(Node):

    def __init__(self):
        super().__init__('sub_node')
        rclpy.logging.set_logger_level('sub_node', rclpy.logging.LoggingSeverity.DEBUG)
        
        self.group = ReentrantCallbackGroup()
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10, callback_group=self.group)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=self.group)
        self.timer = self.create_timer(0.5, self.timer_callback, callback_group=self.group)
        self.laser_msg = LaserScan()
        self.odom_msg = Odometry()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.publisher = PublisherClass()

    def odom_callback(self, msg):
        self.get_logger().debug("Odom CallBack")
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = self.euler_from_quaternion (orientation_list)

    def scan_callback(self, msg):
        self.get_logger().debug("Scan CallBack")
        self.laser_msg = msg

    def get_front_laser(self):
        return self.laser_msg.ranges[360]

    def get_yaw(self):
        return self.yaw

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to Euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Below should be replaced when porting for ROS2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def timer_callback(self):
        self.get_logger().info("Timer CallBack")
        try:
            self.get_logger().warning(">>>>>>>>>>>>>>RANGES Value="+str(self.laser_msg.ranges[360]))
            if not self.laser_msg.ranges[360] < 0.8:                
                self.publisher.move_straight()
            else:                
                self.publisher.stop_robot()
                self.publisher.rotate()
        except:
            pass

class PublisherClass(Node):

    def __init__(self, seconds_sleeping=10):
        super().__init__('pub_node')
        self._seconds_sleeping = seconds_sleeping
        rclpy.logging.set_logger_level('pub_node', rclpy.logging.LoggingSeverity.DEBUG)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist()
    
    def stop_robot(self):
        self.get_logger().info("Ex2 MOVE STOP")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.vel_pub.publish(self.cmd)

    def move_straight(self):
        self.get_logger().info("Ex2 MOVE STRAIGHT")
        self.cmd.linear.x = 0.15
        self.cmd.angular.z = 0.0
        self.vel_pub.publish(self.cmd)
    
    def rotate(self):
        self.get_logger().info("Ex2 MOVE ROTATE")
        self.cmd.angular.z = -0.2
        self.cmd.linear.x = 0.0
        self.get_logger().info("PUBLISH COMMAND...")
        self.vel_pub.publish(self.cmd)        
        self.get_logger().info("PUBLISH COMMAND...FINISHED")
        self.get_logger().info("Ex2 Rotating for "+str(self._seconds_sleeping)+" seconds")
        for i in range(self._seconds_sleeping):
            self.get_logger().info("Ex2 SLEEPING=="+str(i)+" seconds")
            time.sleep(1)
        
        self.stop_robot()
    

def main(args=None):
    rclpy.init(args=args)
    try:
        subs_node = SubscriberClass()
        pub_node = PublisherClass()
        
        executor = SingleThreadedExecutor()
        executor.add_node(subs_node)
        executor.add_node(pub_node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            subs_node.destroy_node()
            pub_node.destroy_node()

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```