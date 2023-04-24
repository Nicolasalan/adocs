# Qualidade de Serviço (QoS)

O que eles são
Você já ouviu o termo QoS várias vezes neste curso, mas o que é? E como isso se relaciona com o ROS2, enquanto no ROS1 não existe?

No ROS1, o TCP foi usado para transportar mensagens.

Este protocolo de transporte não deve ser usado em redes com perdas, como Wifi ou sistemas precários.

Isso não permitiu que os engenheiros usassem o ROS em elementos críticos do sistema ou em ambientes do mundo real.

No entanto, tudo isso mudou com o ROS2 usando UDP como transporte e usando DDS. Agora, você pode decidir o nível de confiabilidade de um nó e decidir o que fazer com base nisso.

Todos os diferentes DDS suportados pelo ROS2 fornecem controle preciso sobre QoS. por exemplo, isso pode ser configurado em Editores e Assinantes de tópicos.

No entanto, para se comunicar entre eles, sua QoS deve ser compatível, o que é uma fonte de problemas no ROS2.

## Como alterá-los e compatibilidade de QoS

Crie dois scripts dentro de um novo pacote que você usará para testar todos os conceitos da unidade de QoS. Neste caso, reveja o conceito de QoS INCOMPATÍVEL:

### **0. Crie um novo pacote** chamado `qos_tests_pkg`

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python qos_tests_pkg --dependencies rclpy std_msgs
cd ~/ros2_ws/src/qos_tests_pkg
```
### **1. Crie dois novos arquivos** chamados `publisher_dds_custom_qos.py` e `Subscriber_dds_custom_minimal_qos.py` dentro da pasta `dds_tests_pkg` dentro do pacote recém-criado.

```bash
cd ~/ros2_ws/src/qos_tests_pkg
touch qos_tests_pkg/subscriber_custom_minimal_qos.py
touch qos_tests_pkg/publisher_custom_minimal_qos.py
```
No arquivo que você acabou de criar, copie o seguinte código:

> subscriber_custom_minimal_qos.py

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# import QoS library, to set the correct profile and reliability.
from rclpy.qos import ReliabilityPolicy, QoSProfile


class SubscriberQoS(Node):

    def __init__(self):

        super().__init__('subscriber_qos_obj')

        # create the subscriber object
        self.subscriber = self.create_subscription(
            String,
            '/qos_test',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def listener_callback(self, msg):
        self.get_logger().info("Data Received ="+str(msg.data))


def main(args=None):
    rclpy.init(args=args)
    sub_qos_obj = SubscriberQoS()
    rclpy.spin(sub_qos_obj)
    sub_qos_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
Aqui, defina a `confiabilidade=ReliabilityPolicy.RELIABLE`. Isso significa que você está forçando esse assinante a receber todas as mensagens enviadas pelo editor.

> publisher_custom_minimal_qos.py

Neste script, revise conceitos importantes que são úteis para ROS2 em Python, especialmente relacionados a QoS:
```python
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
from rclpy.qos_event import PublisherEventCallbacks
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSLivelinessPolicy
from rclpy.qos import QoSReliabilityPolicy


class PublisherQoS(Node):

    def __init__(self, qos_profile, node_name="publisher_qos_obj"):

        super().__init__(node_name)
        # create the publisher object
        #  create_publisher(msg_type, topic, qos_profile, *, callback_group=None, event_callbacks=None)
        # INFO: https://docs.ros2.org/foxy/api/rclpy/api/node.html

        rclpy.logging.set_logger_level(
            node_name, rclpy.logging.LoggingSeverity.INFO)

        event_callbacks = PublisherEventCallbacks(
            incompatible_qos=self.incompatible_qos_clb)

        self.publisher_ = self.create_publisher(msg_type=String,
                                                topic='/qos_test',
                                                qos_profile=qos_profile,
                                                event_callbacks=event_callbacks)

        # This is the unique ID for each message that will be sent
        self.msgs_id = 0
        #self.current_time = self.get_clock().now()
        self.current_time_s = 0
        self.current_time_ns = 0
        # define the timer for 0.5 seconds
        timer_period = 0.5
        # create a timer sending two parameters:
        # - the duration between two Callbacks (0.5 seconds)
        # - the Timer function (timer_callback)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def incompatible_qos_clb(self, event):
        """
        This is the Callback that will be executed when the event of **Incompatible QoS** is
        triggered.
        """
        self.get_logger().error("A subscriber is asking for an INCOMPATIBLE QoS Triggered!")
        self.get_logger().error(str(event.last_policy_kind))
        self.get_logger().error("############################")

    def timer_callback(self):
        # Here you have the Callback method
        msg = String()
        test_time = self.get_clock().now()
        self.current_time_s, self.current_time_ns = test_time.seconds_nanoseconds()
        time_str = str(self.current_time_s)+","+str(self.current_time_ns)
        dds_msg_str = str(self.msgs_id)+":"+time_str
        msg.data = dds_msg_str
        # Publish the message to the topic
        self.publisher_.publish(msg)
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % msg)

        self.msgs_id += 1


def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-reliability',
        type=str,
        choices=['best_effort', 'reliable'],
        help='Select Policy for reliability, use ros2 run dds_tests_pkg publisher_dds_custom_qos_exe -reliability best_effort|reliable')
    return parser


def main(args=None):

    # Parse the arguments
    parser = get_parser()
    parsed_args = parser.parse_args()

    # Configuration variables
    reliability = parsed_args.reliability
    print(reliability)
    qos_profile_publisher = QoSProfile(depth=10)

    # Options  QoSDurabilityPolicy.VOLATILE, QoSDurabilityPolicy.TRANSIENT_LOCAL,
    qos_profile_publisher.durability = QoSDurabilityPolicy.VOLATILE

    qos_profile_publisher.deadline = Duration(seconds=2)

    # Options QoSLivelinessPolicy.MANUAL_BY_TOPIC, QoSLivelinessPolicy.AUTOMATIC
    qos_profile_publisher.liveliness = QoSLivelinessPolicy.AUTOMATIC

    qos_profile_publisher.liveliness_lease_duration = Duration(seconds=2)

    # Options: QoSReliabilityPolicy.RELIABLE, QoSReliabilityPolicy.BEST_EFFORT
    if reliability == "reliable":
        qos_profile_publisher.reliability = QoSReliabilityPolicy.RELIABLE
    else:
        qos_profile_publisher.reliability = QoSReliabilityPolicy.BEST_EFFORT

    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    pub_qos_obj = PublisherQoS(qos_profile_publisher)
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(pub_qos_obj)
    # Explicity destroy the node
    pub_qos_obj.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
### 2. Modifique o `setup.py` para incluir o arquivo de inicialização que você criou. Em seguida, adicione os pontos de entrada ao executável.

```python
from setuptools import setup
import os
from glob import glob

package_name = 'qos_tests_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'publisher_custom_minimal_qos_exe = qos_tests_pkg.publisher_custom_minimal_qos:main',
            'subscriber_custom_minimal_qos_exe = qos_tests_pkg.subscriber_custom_minimal_qos:main',          
        ],
    },
)
```
Agora, compile.
```bash
cd ~/ros2_ws/
colcon build --symlink-install --packages-select qos_tests_pkg
source install/setup.bash
```
Agora, execute esses scripts e veja o que acontece:

> Observe que você está CONFIGURANDO A versão DDS explicitamente. Isso ocorre porque cada implementação DDS tem algumas QoS que suportam e outras que não. Dessa forma, você garante que está usando o ciclone porque é a versão padrão no Galactic.

```bash
cd ~/ros2_ws/
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run qos_tests_pkg publisher_custom_minimal_qos_exe -reliability reliable
```

> Output

```bash
reliable
[INFO] [1644578723.014033825] [publisher_qos_obj]: Publishing: "std_msgs.msg.String(data='0:1644578722,985456932')"
[INFO] [1644578723.486186513] [publisher_qos_obj]: Publishing: "std_msgs.msg.String(data='1:1644578723,485332074')"
[INFO] [1644578723.986142873] [publisher_qos_obj]: Publishing: "std_msgs.msg.String(data='2:1644578723,985345254')"
[INFO] [1644578724.486645546] [publisher_qos_obj]: Publishing: "std_msgs.msg.String(data='3:1644578724,485467771')"
[INFO] [1644578724.986427990] [publisher_qos_obj]: Publishing: "std_msgs.msg.String(data='4:1644578724,985333069')"
[INFO] [1644578725.486563859] [publisher_qos_obj]: Publishing: "std_msgs.msg.String(data='5:1644578725,485435341')"
[INFO] [1644578725.986608071] [publisher_qos_obj]: Publishing: "std_msgs.msg.String(data='6:1644578725,985460097')"
[INFO] [1644578726.486474454] [publisher_qos_obj]: Publishing: "std_msgs.msg.String(data='7:1644578726,485332301')"
[INFO] [1644578726.986147983] [publisher_qos_obj]: Publishing: "std_msgs.msg.String(data='8:1644578726,985357784')"
```

```bash
cd ~/ros2_ws/
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run qos_tests_pkg subscriber_custom_minimal_qos_exe
```

> Output 

```bash
[INFO] [1644578723.015739998] [subscriber_qos_obj]: Data Received =0:1644578722,985456932
[INFO] [1644578723.486751033] [subscriber_qos_obj]: Data Received =1:1644578723,485332074
[INFO] [1644578723.986497548] [subscriber_qos_obj]: Data Received =2:1644578723,985345254
[INFO] [1644578724.486954816] [subscriber_qos_obj]: Data Received =3:1644578724,485467771
[INFO] [1644578724.986790852] [subscriber_qos_obj]: Data Received =4:1644578724,985333069
[INFO] [1644578725.486864984] [subscriber_qos_obj]: Data Received =5:1644578725,485435341
[INFO] [1644578725.986921274] [subscriber_qos_obj]: Data Received =6:1644578725,985460097
[INFO] [1644578726.486804679] [subscriber_qos_obj]: Data Received =7:1644578726,485332301
[INFO] [1644578726.986422874] [subscriber_qos_obj]: Data Received =8:1644578726,985357784
```

Portanto, não há problemas aqui ao usar um QoS COMPATÍVEL. No entanto, o que acontece se você usar um QoS INCOMPATÍVEL?

```bash
cd ~/ros2_ws/
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run qos_tests_pkg publisher_custom_minimal_qos_exe -reliability best_effort
```

> Output

```bash
best_effort
[ERROR] [1644578813.964901713] [publisher_qos_obj]: A subscriber is asking for an INCOMPATIBLE QoS Triggered!
[ERROR] [1644578813.965861273] [publisher_qos_obj]: rmw_qos_policy_kind_t.RMW_QOS_POLICY_RELIABILITY
[ERROR] [1644578813.966438368] [publisher_qos_obj]: ############################
[INFO] [1644578814.439657635] [publisher_qos_obj]: Publishing: "std_msgs.msg.String(data='0:1644578814,438408983')"
[INFO] [1644578814.939157191] [publisher_qos_obj]: Publishing: "std_msgs.msg.String(data='1:1644578814,938248853')"
[INFO] [1644578815.439442102] [publisher_qos_obj]: Publishing: "std_msgs.msg.String(data='2:1644578815,438273310')"
[INFO] [1644578815.939210872] [publisher_qos_obj]: Publishing: "std_msgs.msg.String(data='3:1644578815,938319227')"
[INFO] [1644578816.439078646] [publisher_qos_obj]: Publishing: "std_msgs.msg.String(data='4:1644578816,438258510')"
[INFO] [1644578816.939358849] [publisher_qos_obj]: Publishing: "std_msgs.msg.String(data='5:1644578816,938258582')"
[INFO] [1644578817.439420153] [publisher_qos_obj]: Publishing: "std_msgs.msg.String(data='6:1644578817,438366411')"
[INFO] [1644578817.939196207] [publisher_qos_obj]: Publishing: "std_msgs.msg.String(data='7:1644578817,938358924')"
```

Aqui, DOIS eventos acontecem:

No publicador, o evento de incompatível_qos é TRIGGERED, mostrando a mensagem dentro do Callback incompatível_qos_clb.

No assinante, você recebe o aviso de QoS incompatível.

O motivo é que a QoS do publicador e do assinante são INCOMPATÍVEIS:

Confiabilidade de QoS do Editor = Melhor_Esforço
Confiabilidade de QoS do Assinante=Confiável
Ele também informa o QoS que é incompatível, pelo menos o último:

Última apólice incompatível: CONFIABILIDADE
Esta configuração é INCOMPATÍVEL.

No entanto, como você sabe quais configurações são compatíveis?

### Políticas Padrão
Primeiro, você precisa conhecer as políticas padrão ao criar um assinante e um editor.

* Editores, assinantes
     * `history` = “manter por último”
     * `queue size`  = 10
     * `reliability` = confiável
     * `durability` = volátil
     * `liveliness` = padrão do sistema.
     * `Deadline`, vida útil e durações de concessão são definidos como grandes números = 9223372036854775807 nanossegundos.

## Exemplos de como usar as diferentes Políticas
Veja os exemplos para ver como você pode usar as diferentes políticas de QoS.

### Durabilidade
A durabilidade regula se uma mensagem publicada pelo editor permanece lá para sempre, para que os assinantes possam lê-la mesmo quando se inscreverem após a publicação da mensagem. Isso, por exemplo, é como robot_description ou parâmetros globais devem funcionar.

Você tem duas configurações:

* Local transitório: o editor é responsável por fazer com que as mensagens persistam no tempo para os assinantes que se juntam depois que o editor publica a mensagem.
* Volátil: Você não fará nada para que as mensagens persistam.

```bash
cd ~/ros2_ws/src/qos_tests_pkg
touch qos_tests_pkg/subscriber_durability.py
touch qos_tests_pkg/publisher_durability.py
```

> subscriber_durability.py

```python
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy


class SubscriberQoS(Node):

    def __init__(self, qos_profile, node_name="subscriber_qos_obj"):

        super().__init__(node_name)

        rclpy.logging.set_logger_level(
            node_name,
            rclpy.logging.LoggingSeverity.INFO)

        event_callbacks = SubscriptionEventCallbacks(
            incompatible_qos=self.incompatible_qos_clb)

        self.subscriber = self.create_subscription(
            msg_type=String,
            topic='/qos_test',
            callback=self.listener_callback,
            qos_profile=qos_profile,
            event_callbacks=event_callbacks)

    def listener_callback(self, msg):
        self.get_logger().info("Data Received ="+str(msg.data))

    def incompatible_qos_clb(self, event):
        """
        This is the Callback that will be executed when the event of **Incompatible QoS** is
        triggered.
        """
        self.get_logger().error("SUBSCRIBER::: INCOMPATIBLE QoS Triggered!")
        self.get_logger().error(str(event.last_policy_kind))
        self.get_logger().error("############################")


def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-durability',
        type=str,
        choices=['transient_local', 'volatile'],
        help='Select Policy for durability, use ros2 run qos_tests_pkg name_of_exe -durability transient_local|volatile')
    return parser


def main(args=None):

    parser = get_parser()
    parsed_args = parser.parse_args()

    # Depth of History is Compulsory
    qos_profile_subscriber = QoSProfile(depth=1)

    # Options  QoSDurabilityPolicy.VOLATILE, QoSDurabilityPolicy.TRANSIENT_LOCAL,
    durability = parsed_args.durability
    print(durability)
    if durability == "transient_local":
        qos_profile_subscriber.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    else:
        # Leave the one by default, which is VOLATILE
        pass

    rclpy.init(args=args)
    sub_qos_obj = SubscriberQoS(qos_profile_subscriber)
    rclpy.spin(sub_qos_obj)
    sub_qos_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
Aqui você está definindo a `confiabilidade=ReliabilityPolicy.RELIABLE`. Isso significa que você está forçando esse assinante a receber cada mensagem enviada pelo editor.

> publisher_durability.py

Neste script, você revisará vários conceitos essenciais que são úteis para o ROS2 em Python, especialmente relacionados a QoS:

```python
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos_event import PublisherEventCallbacks
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy


class PublisherQoS(Node):

    def __init__(self, qos_profile, node_name="publisher_qos_obj"):

        super().__init__(node_name)
        # create the publisher object
        #  create_publisher(msg_type, topic, qos_profile, *, callback_group=None, event_callbacks=None)
        # INFO: https://docs.ros2.org/galactic/api/rclpy/api/node.html

        rclpy.logging.set_logger_level(
            node_name, rclpy.logging.LoggingSeverity.INFO)

        event_callbacks = PublisherEventCallbacks(
            incompatible_qos=self.incompatible_qos_clb)

        self.publisher_ = self.create_publisher(msg_type=String,
                                                topic='/qos_test',
                                                qos_profile=qos_profile,
                                                event_callbacks=event_callbacks)

    def incompatible_qos_clb(self, event):
        """
        This is the Callback that will be executed when the event of **Incompatible QoS** is
        triggered.
        """
        self.get_logger().error("PUBLISHER::: INCOMPATIBLE QoS Triggered!")
        self.get_logger().error(str(event.last_policy_kind))
        self.get_logger().error("############################")

    def publish_one_message(self):
        # Here you have the Callback method
        msg = String()
        test_time = self.get_clock().now()
        self.current_time_s, self.current_time_ns = test_time.seconds_nanoseconds()
        time_str = str(self.current_time_s)+","+str(self.current_time_ns)
        msg.data = time_str
        self.publisher_.publish(msg)
        self.get_logger().info('Published: "%s"' % msg)


def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-durability',
        type=str,
        choices=['transient_local', 'volatile'],
        help='Select Policy for durability, use ros2 run qos_tests_pkg name_of_exe -durability transient_local|volatile')
    return parser


def main(args=None):

    parser = get_parser()
    parsed_args = parser.parse_args()

    # Depth of History is Compulsory
    qos_profile_publisher = QoSProfile(depth=1)

    # Options  QoSDurabilityPolicy.VOLATILE, QoSDurabilityPolicy.TRANSIENT_LOCAL,
    durability = parsed_args.durability
    print(durability)
    if durability == "transient_local":
        qos_profile_publisher.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    else:
        # Leave the one by default, which is VOLATILE
        pass

    rclpy.init(args=args)
    pub_qos_obj = PublisherQoS(qos_profile_publisher)
    pub_qos_obj.publish_one_message()
    rclpy.spin(pub_qos_obj)
    pub_qos_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

> setup.py

```python
from setuptools import setup

package_name = 'qos_tests_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='duckfrost@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            
            'subscriber_durability_exe = qos_tests_pkg.subscriber_durability:main',
            'publisher_durability_exe = qos_tests_pkg.publisher_durability:main',
        ],
    },
)
```
Agora teste o código, usando os scripts Python e os comandos shell:

```bash
cd ~/ros2_ws/
colcon build --symlink-install --packages-select qos_tests_pkg
source install/setup.bash
```

Execute os scripts:
```bash
ros2 run qos_tests_pkg publisher_durability_exe -durability transient_local
```

> Output

```bash
ransient_local
[INFO] [1644605326.998430672] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644605326,985073117')"
```
Agora, este assinante deve receber a mensagem mesmo que ela tenha sido iniciada depois que o publicador publicou a mensagem:
```bash
ros2 run qos_tests_pkg subscriber_durability_exe -durability transient_local
```

> Output 

```bash
transient_local
[INFO] [1644605352.435077445] [subscriber_qos_obj]: Data Received =1644605326,985073117
```
Você também pode usar um comando de terminal simples. Você define o comando echo com o QoS apropriado:

```bash
ros2 topic echo --qos-durability transient_local --qos-reliability reliable /qos_test
```
Separe o confiável do local transitório; caso contrário, não funcionará. Este problema no Galactic é resolvido na próxima versão do ROS2.

```bash
ros2 topic echo --qos-durability transient_local  /qos_test
```

### Prazo final
Agora considere o máximo que deve passar entre uma publicação de mensagem e outra para considerar o tópico saudável. Isso é útil para comandos de controle, como cmd_vel, ou outros comandos mais críticos que precisam ser regulares, como detecção de obstáculos, por exemplo.

A variável o regula:

* Duração: Tempo entre as mensagens subsequentes publicadas.

Crie alguns exemplos para ver como você regularia isso:

```bash
cd ~/ros2_ws/src/qos_tests_pkg
touch qos_tests_pkg/subscriber_deadline.py
touch qos_tests_pkg/publisher_deadline.py
```

> subscriber_deadline.py

```python
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.qos import QoSProfile

from rclpy.duration import Duration


class SubscriberQoS(Node):

    def __init__(self, qos_profile, node_name="subscriber_qos_obj"):

        super().__init__(node_name)

        rclpy.logging.set_logger_level(
            node_name,
            rclpy.logging.LoggingSeverity.INFO)

        event_callbacks = SubscriptionEventCallbacks(
            incompatible_qos=self.incompatible_qos_clb,
            deadline=self.deadline_qos_clb)

        self.subscriber = self.create_subscription(
            msg_type=String,
            topic='/qos_test',
            callback=self.listener_callback,
            qos_profile=qos_profile,
            event_callbacks=event_callbacks)

    def listener_callback(self, msg):
        self.get_logger().info("Data Received ="+str(msg.data))

    def incompatible_qos_clb(self, event):
        """
        This is the Callback that will be executed when the event of **Incompatible QoS** is
        triggered.
        """
        self.get_logger().error("SUBSCRIBER::: INCOMPATIBLE QoS Triggered!")
        self.get_logger().error(str(event.last_policy_kind))
        self.get_logger().error("############################")

    def deadline_qos_clb(self, event):
        """
        Triggered when the deadline is achieved
        """
        self.get_logger().error("PUBLISHER:::  Deadline Triggered!")


def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-deadline',
        type=float,
        help='Select Policy for deadline in seconds, use ros2 run qos_tests_pkg name_of_exe -deadline 1.350')
    return parser


def main(args=None):

    parser = get_parser()
    parsed_args = parser.parse_args()

    # Depth of History is Compulsory
    qos_profile_subscriber = QoSProfile(depth=1)

    deadline_seconds = float(parsed_args.deadline)
    deadline = Duration(seconds=deadline_seconds)
    print("deadline=="+str(deadline))
    qos_profile_subscriber.deadline = deadline

    rclpy.init(args=args)
    sub_qos_obj = SubscriberQoS(qos_profile_subscriber)
    rclpy.spin(sub_qos_obj)
    sub_qos_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

> publisher_deadline.py

```python
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos_event import PublisherEventCallbacks
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy

import time
from rclpy.duration import Duration


class PublisherQoS(Node):

    def __init__(self, qos_profile, node_name="publisher_qos_obj"):

        super().__init__(node_name)

        rclpy.logging.set_logger_level(
            node_name, rclpy.logging.LoggingSeverity.INFO)

        event_callbacks = PublisherEventCallbacks(
            incompatible_qos=self.incompatible_qos_clb,
            deadline=self.deadline_qos_clb)

        self.publisher_ = self.create_publisher(msg_type=String,
                                                topic='/qos_test',
                                                qos_profile=qos_profile,
                                                event_callbacks=event_callbacks)

        # Create a timer
        self.timer_period = 1.0
        self.swap_state_time = 5.0
        self.time_pause = 2.0
        self.counter = 0

        self.create_timer(self.timer_period, self.timer_callback)

    def incompatible_qos_clb(self, event):
        """
        This is the Callback that will be executed when the event of **Incompatible QoS** is
        triggered.
        """
        self.get_logger().error("PUBLISHER::: INCOMPATIBLE QoS Triggered!")
        self.get_logger().error(str(event.last_policy_kind))
        self.get_logger().error("############################")

    def deadline_qos_clb(self, event):
        """
        Triggered when the deadline is achieved
        """
        self.get_logger().error("PUBLISHER:::  Deadline Triggered!")

    def publish_one_message(self):
        # Here you have the callback method
        msg = String()
        test_time = self.get_clock().now()
        self.current_time_s, self.current_time_ns = test_time.seconds_nanoseconds()
        time_str = str(self.current_time_s)+","+str(self.current_time_ns)
        msg.data = time_str
        self.publisher_.publish(msg)
        self.get_logger().info('Published: "%s"' % msg)

    def timer_callback(self):
        # print a ROS2 log on the terminal with a great message!

        if self.counter > int(self.swap_state_time / self.timer_period):
            delta = 0.1
            range_steps = int(self.time_pause / delta)
            for i in range(range_steps):
                time.sleep(delta)
                self.get_logger().info("Paused ="+str(i*delta)+"/"+str(self.time_pause))
            self.counter = 0
        else:
            self.publish_one_message()
            self.counter += 1
            self.get_logger().info("Counter ="+str(self.counter))


def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-deadline',
        type=float,
        help='Select Policy for deadline in seconds, use ros2 run qos_tests_pkg name_of_exe -deadline 1.350')
    return parser


def main(args=None):

    parser = get_parser()
    parsed_args = parser.parse_args()

    # Depth of History is Compulsory
    qos_profile_publisher = QoSProfile(depth=1)

    deadline_seconds = float(parsed_args.deadline)
    deadline = Duration(seconds=deadline_seconds)
    print("deadline=="+str(deadline))
    qos_profile_publisher.deadline = deadline

    rclpy.init(args=args)
    pub_qos_obj = PublisherQoS(qos_profile_publisher)
    pub_qos_obj.publish_one_message()
    rclpy.spin(pub_qos_obj)
    pub_qos_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

> setup.py

Aqui está o mínimo para este exemplo funcionar. No entanto, você também pode deixar os executáveis anteriores.

```bash
from setuptools import setup

package_name = 'qos_tests_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='duckfrost@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'subscriber_deadline_exe = qos_tests_pkg.subscriber_deadline:main',
            'publisher_deadline_exe = qos_tests_pkg.publisher_deadline:main'
        ],
    },
)
```
Compile para executar os novos executáveis:

```bash
cd ~/ros2_ws/
colcon build --symlink-install --packages-select qos_tests_pkg
source install/setup.bash
```
Execute os scripts em condições de funcionamento, como exemplo. Condição de trabalho significa que:

* O publisher tem um prazo que supera o tempo que leva para publicar suas mensagens.
* O assinante tem um prazo que supera o tempo do editor para publicar normalmente E durante a fase de pausa no código.
* Ambos possuem valores de QoS compatíveis (veja a tabela mostrada anteriormente).

```bash
ros2 run qos_tests_pkg publisher_deadline_exe -deadline 10.0
```

```bash
ros2 run qos_tests_pkg subscriber_deadline_exe -deadline 10.0
```

> Output 1

```bash
deadline==Duration(nanoseconds=10000000000)
[INFO] [1644936043.662228672] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644936043,647624987')"
[INFO] [1644936044.648765369] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644936044,648061429')"
[INFO] [1644936044.649647590] [publisher_qos_obj]: Counter =1
[INFO] [1644936045.648750958] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644936045,648067137')"
[INFO] [1644936045.649631783] [publisher_qos_obj]: Counter =2
[INFO] [1644936046.648815753] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644936046,647949835')"
[INFO] [1644936046.649476575] [publisher_qos_obj]: Counter =3
[INFO] [1644936047.648672771] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644936047,647940313')"
[INFO] [1644936047.649307089] [publisher_qos_obj]: Counter =4
[INFO] [1644936048.648772383] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644936048,648042781')"
[INFO] [1644936048.649380699] [publisher_qos_obj]: Counter =5
[INFO] [1644936049.648826664] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644936049,648102316')"
[INFO] [1644936049.649409283] [publisher_qos_obj]: Counter =6
[INFO] [1644936050.748734377] [publisher_qos_obj]: Paused =0.0/2.0
[INFO] [1644936050.849589211] [publisher_qos_obj]: Paused =0.1/2.0
[INFO] [1644936050.950404186] [publisher_qos_obj]: Paused =0.2/2.0
[INFO] [1644936051.051182319] [publisher_qos_obj]: Paused =0.30000000000000004/2.0
[INFO] [1644936051.152053497] [publisher_qos_obj]: Paused =0.4/2.0
[INFO] [1644936051.252723450] [publisher_qos_obj]: Paused =0.5/2.0
[INFO] [1644936051.353412801] [publisher_qos_obj]: Paused =0.6000000000000001/2.0
[INFO] [1644936051.454263050] [publisher_qos_obj]: Paused =0.7000000000000001/2.0
[INFO] [1644936051.555149310] [publisher_qos_obj]: Paused =0.8/2.0
[INFO] [1644936051.655982300] [publisher_qos_obj]: Paused =0.9/2.0
[INFO] [1644936051.756735299] [publisher_qos_obj]: Paused =1.0/2.0
[INFO] [1644936051.857541979] [publisher_qos_obj]: Paused =1.1/2.0
[INFO] [1644936051.958319656] [publisher_qos_obj]: Paused =1.2000000000000002/2.0
[INFO] [1644936052.059184485] [publisher_qos_obj]: Paused =1.3/2.0
[INFO] [1644936052.159936629] [publisher_qos_obj]: Paused =1.4000000000000001/2.0
[INFO] [1644936052.260631850] [publisher_qos_obj]: Paused =1.5/2.0
[INFO] [1644936052.361722072] [publisher_qos_obj]: Paused =1.6/2.0
[INFO] [1644936052.462475582] [publisher_qos_obj]: Paused =1.7000000000000002/2.0
[INFO] [1644936052.563326299] [publisher_qos_obj]: Paused =1.8/2.0
[INFO] [1644936052.664155447] [publisher_qos_obj]: Paused =1.9000000000000001/2.0
[INFO] [1644936052.665534631] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644936052,664860538')"
[INFO] [1644936052.666173215] [publisher_qos_obj]: Counter =1
[INFO] [1644936053.648618315] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644936053,647928872')"
[INFO] [1644936053.649236053] [publisher_qos_obj]: Counter =2
```

> Output 2

```bash
deadline==Duration(nanoseconds=10000000000)
[INFO] [1644936043.662486552] [subscriber_qos_obj]: Data Received =1644936043,647624987
[INFO] [1644936044.649069145] [subscriber_qos_obj]: Data Received =1644936044,648061429
[INFO] [1644936045.649059219] [subscriber_qos_obj]: Data Received =1644936045,648067137
[INFO] [1644936046.649305336] [subscriber_qos_obj]: Data Received =1644936046,647949835
[INFO] [1644936047.649032866] [subscriber_qos_obj]: Data Received =1644936047,647940313
[INFO] [1644936048.648978854] [subscriber_qos_obj]: Data Received =1644936048,648042781
[INFO] [1644936049.649103841] [subscriber_qos_obj]: Data Received =1644936049,648102316
[INFO] [1644936052.665971148] [subscriber_qos_obj]: Data Received =1644936052,664860538
[INFO] [1644936053.648887735] [subscriber_qos_obj]: Data Received =1644936053,647928872
```
Você pode ver que o assinante não tem erro, mesmo quando o editor não publicou nada por aproximadamente 2 segundos. Isso porque o assinante tinha 10,0 segundos do deadline, que está dentro da margem.

E o editor também não tem erro porque está publicando algo com um período inferior ao seu deadline de 10,0 segundos.

Teste diferentes valores dos deadlines para ver os diferentes comportamentos:

**1. O editor não cumpre o prazo:**

     * Aqui você deve definir um prazo menor que o tempo que o editor leva para publicar sua mensagem.
**2. O assinante não recebe a mensagem do publicador a tempo:**

     * Defina o prazo para o assinante menor que o tempo do publicador para publicar cada mensagem.
     * Se o prazo do assinante for pequeno o suficiente, isso acionará o evento Callback no loop de pausa ou mesmo no loop de publicação normal.

**3. Os assinantes e editores têm valores de QoS incompatíveis.**

     * Aqui você tem que dar uma olhada na tabela de compatibilidades.

Aqui você tem comandos e as saídas dos diferentes cenários:

### 1. A Editora não cumpre seu prazo:
O editor publica uma mensagem a cada 1,0 segundos. Isso significa que se você definir um prazo menor que 1,0 segundos, ele acionará `deadline_qos_clb`:

```bash
ros2 run qos_tests_pkg publisher_deadline_exe -deadline 0.99
```

> Output 

```bash
deadline==Duration(nanoseconds=990000000)
[INFO] [1644945024.262185275] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644945024,248831403')"
[ERROR] [1644945025.239750790] [publisher_qos_obj]: PUBLISHER:::  Deadline Triggered!
[INFO] [1644945025.249586808] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644945025,249040140')"
[INFO] [1644945025.250109844] [publisher_qos_obj]: Counter =1
[ERROR] [1644945026.239969321] [publisher_qos_obj]: PUBLISHER:::  Deadline Triggered!
[INFO] [1644945026.250107663] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644945026,249161667')"
[INFO] [1644945026.250936248] [publisher_qos_obj]: Counter =2
[ERROR] [1644945027.240079300] [publisher_qos_obj]: PUBLISHER:::  Deadline Triggered!
[INFO] [1644945027.249587302] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644945027,249039825')"
[INFO] [1644945027.250048127] [publisher_qos_obj]: Counter =3
[ERROR] [1644945028.239945480] [publisher_qos_obj]: PUBLISHER:::  Deadline Triggered!
```

Como você pode ver, mesmo com deadline de 0,99, ele já dispara.

Você também pode fazer com que falhe na fase de pausa:

```bash
ros2 run qos_tests_pkg publisher_deadline_exe -deadline 1.0
```

> Output

```bash
[INFO] [1645093546.096753656] [publisher_qos_obj]: Paused =1.4000000000000001/2.0
[INFO] [1645093546.197853739] [publisher_qos_obj]: Paused =1.5/2.0
[INFO] [1645093546.298767356] [publisher_qos_obj]: Paused =1.6/2.0
[INFO] [1645093546.399664744] [publisher_qos_obj]: Paused =1.7000000000000002/2.0
[INFO] [1645093546.500553386] [publisher_qos_obj]: Paused =1.8/2.0
[INFO] [1645093546.601426722] [publisher_qos_obj]: Paused =1.9000000000000001/2.0
[ERROR] [1645093546.602328329] [publisher_qos_obj]: PUBLISHER:::  Deadline Triggered!
```

> Observe que o retorno de chamada para o prazo é executado em 2,0 segundos, o que não deveria ser porque o prazo foi definido como 1,0 segundos. O motivo é que você não está usando multithreading aqui, o que faz com que você tenha apenas UM thread por padrão e, portanto, o Callback só será executado após o término do timer_callback.


### 2. O assinante não recebe a mensagem do publicador a tempo:
Nesse caso, estabeleça um prazo maior que a taxa normal de publicação, mas menor que o tempo que ela fica pausada.

```bash
ros2 run qos_tests_pkg publisher_deadline_exe -deadline 1.2
```
Segundo terminal:
```bash
ros2 run qos_tests_pkg subscriber_deadline_exe -deadline 1.8
```
 
> Output

```bash
deadline==Duration(nanoseconds=1500000000)
[INFO] [1644945268.922918882] [subscriber_qos_obj]: Data Received =1644945268,907330118
[INFO] [1644945269.908546026] [subscriber_qos_obj]: Data Received =1644945269,907657960
[INFO] [1644945270.908515029] [subscriber_qos_obj]: Data Received =1644945270,907634661
[INFO] [1644945271.908502764] [subscriber_qos_obj]: Data Received =1644945271,907638918
[INFO] [1644945272.908581267] [subscriber_qos_obj]: Data Received =1644945272,907662620
[INFO] [1644945273.908581865] [subscriber_qos_obj]: Data Received =1644945273,907642513
[INFO] [1644945274.908580555] [subscriber_qos_obj]: Data Received =1644945274,907634612
[ERROR] [1644945276.408579327] [subscriber_qos_obj]: PUBLISHER:::  Deadline Triggered!
[ERROR] [1644945277.908573243] [subscriber_qos_obj]: PUBLISHER:::  Deadline Triggered!
[INFO] [1644945277.924351744] [subscriber_qos_obj]: Data Received =1644945277,923275003
```

Aqui você pode ver isso enquanto:

O editor não está em execução. O deadline não é acionado no assinante.
Ao iniciar a editora, ela só dá o Gatilho de Prazo na editora e no assinante quando a editora está pausada. Isso ocorre porque o editor e o assinante precisam esperar mais de 1,2/1,8 segundos, respectivamente, para obter uma nova mensagem de publicação.

### 3. O assinante e o editor têm valores de QoS incompatíveis:
Se você observar a tabela de compatibilidade, poderá acionar a incompatibilidade de QoS definindo:

prazo do assinante < prazo do editor

```bash
ros2 run qos_tests_pkg publisher_deadline_exe -deadline 1.5
```

> Output 

```bash
[INFO] [1644945489.570006750] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644945489,552461406')"
[ERROR] [1644945489.570866117] [publisher_qos_obj]: PUBLISHER::: INCOMPATIBLE QoS Triggered!
[ERROR] [1644945489.571476981] [publisher_qos_obj]: rmw_qos_policy_kind_t.RMW_QOS_POLICY_DEADLINE
[ERROR] [1644945489.571942584] [publisher_qos_obj]: ############################
[INFO] [1644945490.553480357] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644945490,552765208')"
```

O editor não para, mas dá o erro.
```bash
ros2 run qos_tests_pkg subscriber_deadline_exe -deadline 1.0
```

> Output 
```bash
deadline==Duration(nanoseconds=1000000000)
[ERROR] [1644945489.566753281] [subscriber_qos_obj]: SUBSCRIBER::: INCOMPATIBLE QoS Triggered!
[ERROR] [1644945489.567304443] [subscriber_qos_obj]: rmw_qos_policy_kind_t.RMW_QOS_POLICY_DEADLINE
[ERROR] [1644945489.567773866] [subscriber_qos_obj]: ############################
```

### Vida útil
É definido pela duração, o tempo entre a publicação e o recebimento da mensagem, após o qual a mensagem se torna obsoleta. Isso é importante para dados de sensores como varreduras a laser ou câmeras porque, normalmente, você está interessado em ter os valores mais recentes e os dados que chegam atrasados são inúteis.

Crie alguns exemplos para ver como você regularia isso:

```bash
cd ~/ros2_ws/src/qos_tests_pkg
touch qos_tests_pkg/subscriber_lifespan.py
touch qos_tests_pkg/publisher_lifespan.py
```

Primeiro, a duração do tempo de vida só é útil se você usá-la com confiável e transient_local. A razão é que se você não garantir a recepção da mensagem (Confiável) e não esperar que os assinantes tardios a recebam (transitório_local), não é possível avaliar a duração da mensagem publicada.

É por isso que você define esses QoS no editor e nos assinantes:

> subscriber_lifespan.py

```python
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from rclpy.duration import Duration


class SubscriberQoS(Node):

    def __init__(self, qos_profile, node_name="subscriber_qos_obj"):

        super().__init__(node_name)

        rclpy.logging.set_logger_level(
            node_name,
            rclpy.logging.LoggingSeverity.INFO)

        event_callbacks = SubscriptionEventCallbacks(
            incompatible_qos=self.incompatible_qos_clb)

        # create the subscriber object
        self.subscriber = self.create_subscription(
            msg_type=String,
            topic='/qos_test',
            callback=self.listener_callback,
            qos_profile=qos_profile,
            event_callbacks=event_callbacks)

    def listener_callback(self, msg):
        """
        Parse it and calculate the time passed
        1644948035,948696546
        """
        raw_data = msg.data
        self.get_logger().info("Data Received ="+str(raw_data))
        split_data = raw_data.split(",")
        self.get_logger().info("SPLIT ="+str(split_data))

        seconds = float(split_data[0])
        nseconds = float(split_data[1])

        self.get_logger().info("seconds ="+str(seconds)+", nseconds = "+str(nseconds))

        total_seconds = seconds + nseconds * (10 ** -9)

        self.get_logger().info("total_seconds ="+str(total_seconds))

        # Get time now
        test_time = self.get_clock().now()
        current_time_s, current_time_ns = test_time.seconds_nanoseconds()

        total_current_time = float(current_time_s) + \
            (float(current_time_ns) * (10 ** -9))

        self.get_logger().info("total_current_time ="+str(total_current_time))

        delta = total_current_time - total_seconds

        self.get_logger().info("Message Age ="+str(delta))

    def incompatible_qos_clb(self, event):
        """
        This is the Callback that will be executed when the event of **Incompatible QoS** is
        triggered.
        """
        self.get_logger().error("SUBSCRIBER::: INCOMPATIBLE QoS Triggered!")
        self.get_logger().error(str(event.last_policy_kind))
        self.get_logger().error("############################")


def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-lifespan',
        type=float,
        help='Select Policy for lifespan, use ros2 run qos_tests_pkg name_of_exe -lifespan 3.2')
    return parser


def main(args=None):

    parser = get_parser()
    parsed_args = parser.parse_args()

    # Depth of History is Compulsory
    qos_profile_subscriber = QoSProfile(depth=1)

    lifespan_seconds = float(parsed_args.lifespan)
    lifespan = Duration(seconds=lifespan_seconds)
    print("lifespan=="+str(lifespan))
    qos_profile_subscriber.lifespan = lifespan

    # Reliability set to reliable
    qos_profile_subscriber.reliability = QoSReliabilityPolicy.RELIABLE
    # Durability Set to Transient Local
    qos_profile_subscriber.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

    rclpy.init(args=args)
    sub_qos_obj = SubscriberQoS(qos_profile_subscriber)
    rclpy.spin(sub_qos_obj)
    sub_qos_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

> publisher_lifespan.py

```python
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos_event import PublisherEventCallbacks
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from rclpy.duration import Duration


class PublisherQoS(Node):

    def __init__(self, qos_profile, node_name="publisher_qos_obj"):

        super().__init__(node_name)

        rclpy.logging.set_logger_level(
            node_name, rclpy.logging.LoggingSeverity.INFO)

        event_callbacks = PublisherEventCallbacks(
            incompatible_qos=self.incompatible_qos_clb)

        self.publisher_ = self.create_publisher(msg_type=String,
                                                topic='/qos_test',
                                                qos_profile=qos_profile,
                                                event_callbacks=event_callbacks)

    def incompatible_qos_clb(self, event):
        """
        This is the Callback that will be executed when the event of **Incompatible QoS** is
        triggered.
        """
        self.get_logger().error("PUBLISHER::: INCOMPATIBLE QoS Triggered!")
        self.get_logger().error(str(event.last_policy_kind))
        self.get_logger().error("############################")

    def publish_one_message(self):
        # Here you have the Callback method
        msg = String()
        test_time = self.get_clock().now()
        self.current_time_s, self.current_time_ns = test_time.seconds_nanoseconds()
        time_str = str(self.current_time_s)+","+str(self.current_time_ns)
        msg.data = time_str
        self.publisher_.publish(msg)
        self.get_logger().info('Published: "%s"' % msg)


def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-lifespan',
        type=float,
        help='Select Policy for lifespan, use ros2 run qos_tests_pkg name_of_exe -lifespan 3.2')
    return parser


def main(args=None):

    parser = get_parser()
    parsed_args = parser.parse_args()

    # Depth of History is Compulsory
    qos_profile_publisher = QoSProfile(depth=1)

    lifespan_seconds = float(parsed_args.lifespan)
    lifespan = Duration(seconds=lifespan_seconds)
    print("lifespan=="+str(lifespan))
    qos_profile_publisher.lifespan = lifespan

    # Reliability set to reliable
    qos_profile_publisher.reliability = QoSReliabilityPolicy.RELIABLE
    # Durability Set to Transient Local
    qos_profile_publisher.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

    rclpy.init(args=args)
    pub_qos_obj = PublisherQoS(qos_profile_publisher)
    pub_qos_obj.publish_one_message()
    rclpy.spin(pub_qos_obj)
    pub_qos_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
> setup.py

Aqui está o mínimo para este exemplo funcionar. No entanto, você também pode deixar os executáveis anteriores.

```python
from setuptools import setup

package_name = 'qos_tests_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='duckfrost@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'subscriber_lifespan_exe = qos_tests_pkg.subscriber_lifespan:main',
            'publisher_lifespan_exe = qos_tests_pkg.publisher_lifespan:main'
        ],
    },
)
```

```bash
cd ~/ros2_ws/
colcon build --symlink-install --packages-select qos_tests_pkg
source install/setup.bash
```

Como funciona a vida útil?

* Defina uma data de expiração para as mensagens publicadas pelo editor. Se o assinante receber uma mensagem mais antiga do que o tempo de vida definido no editor, o assinante nem processará esses dados. É como se não chegasse ao assinante.

* No código do assinante, você tem o seguinte:

```python
def listener_callback(self, msg):
    """
    Parse it and calculate the time passed
    1644948035,948696546
    """
    raw_data = msg.data
    self.get_logger().info("Data Received ="+str(raw_data))
    split_data = raw_data.split(",")
    self.get_logger().info("SPLIT ="+str(split_data))

    seconds = float(split_data[0])
    nseconds = float(split_data[1])

    self.get_logger().info("seconds ="+str(seconds)+", nseconds = "+str(nseconds))

    total_seconds = seconds + nseconds * (10 ** -9)

    self.get_logger().info("total_seconds ="+str(total_seconds))

    # Get time now
    test_time = self.get_clock().now()
    current_time_s, current_time_ns = test_time.seconds_nanoseconds()

    total_current_time = float(current_time_s) + \
        (float(current_time_ns) * (10 ** -9))

    self.get_logger().info("total_current_time ="+str(total_current_time))

    delta = total_current_time - total_seconds

    self.get_logger().info("Message Age ="+str(delta))
```

Aqui, calcule o tempo decorrido entre a hora em que a mensagem foi publicada e a hora em que o assinante recebeu a mensagem.

Lembre-se de que, se esse tempo exceder o tempo de vida do editor, ele não acionará o retorno de chamada do assinante. Isso é feito internamente pela estrutura DDS ROS2.

Veja vários exemplos:

funcionando corretamente

```bash
ros2 run qos_tests_pkg publisher_lifespan_exe -lifespan 20.0
```

```bash
lifespan==Duration(nanoseconds=20000000000)
[INFO] [1644949755.683088055] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1644949755,669006598')"
```
Isso deve ser iniciado antes do editor ou nos próximos 20 segundos.
```bash
ros2 run qos_tests_pkg subscriber_lifespan_exe -lifespan 1.0
```

```bash
ifespan==Duration(nanoseconds=1000000000)
[INFO] [1644949766.031053550] [subscriber_qos_obj]: Data Received =1644949755,669006598
[INFO] [1644949766.031555572] [subscriber_qos_obj]: SPLIT =['1644949755', '669006598']
[INFO] [1644949766.032019812] [subscriber_qos_obj]: seconds =1644949755.0, nseconds = 669006598.0
[INFO] [1644949766.032477376] [subscriber_qos_obj]: total_seconds =1644949755.6690066
[INFO] [1644949766.032963361] [subscriber_qos_obj]: total_current_time =1644949766.0325232
[INFO] [1644949766.033457458] [subscriber_qos_obj]: Message Age =10.363516569137573
```

Observe que isso não afeta o valor da vida útil da QoS do assinante.

### Tempo de vida do editor muito restritivo
Para fazer esse teste, você deve iniciar o assinante primeiro; caso contrário, você não pode medir o tempo corretamente.

Isso deve ser iniciado antes do editor ou nos próximos 20 segundos.
```bash
ros2 run qos_tests_pkg subscriber_lifespan_exe -lifespan 1.0
```
Inicie uma vida útil rápida, mas factível pelo seu sistema
```bash
ros2 run qos_tests_pkg publisher_lifespan_exe -lifespan 0.001
```
Como você pode ver, a mensagem Age é de 0,01 segundos, portanto, em teoria, NÃO DEVERIA ter funcionado. No entanto, o assinante foi muito mais rápido do que o tempo que o Callback levou para fazer os cálculos e logar dentro do Callback. Ele recebeu a mensagem em 0,001 segundos de tempo de vida do publicador.

```bash
[INFO] [1644950006.549922391] [subscriber_qos_obj]: Data Received =1644950006,531806174
[INFO] [1644950006.550509748] [subscriber_qos_obj]: SPLIT =['1644950006', '531806174']
[INFO] [1644950006.551090181] [subscriber_qos_obj]: seconds =1644950006.0, nseconds = 531806174.0
[INFO] [1644950006.551656906] [subscriber_qos_obj]: total_seconds =1644950006.5318062
[INFO] [1644950006.552259160] [subscriber_qos_obj]: total_current_time =1644950006.5517163
[INFO] [1644950006.552856942] [subscriber_qos_obj]: Message Age =0.019910097122192383
```
Agora, lance um editor com uma vida útil curta que seu sistema não pode processar essa mensagem a tempo de chegar ao assinante.
```bash
ros2 run qos_tests_pkg publisher_lifespan_exe -lifespan 0.00001
```
Agora o assinante não acionou o Callback.

> Output 

```bash
[NOTHING]
```

### Animação e ArrendamentoDuração
A vivacidade é usada para saber se um publicador ou nó está ativo. Isso significa que ele é publicado a uma taxa regular.

O leaseDuration é o período em que você considera que o publicador tem que dar alguma mensagem ou sinal que está vivo; caso contrário, considere-o morto.

Primeiro, crie os dois arquivos para este exemplo:

```bash
cd ~/ros2_ws/src/qos_tests_pkg
touch qos_tests_pkg/subscriber_liveliness.py
touch qos_tests_pkg/publisher_liveliness.py
```

> subscriber_liveliness.py

```python
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.qos import QoSProfile
from rclpy.qos import QoSLivelinessPolicy

from rclpy.duration import Duration

import time

POLICY_MAP = {
    'AUTOMATIC': QoSLivelinessPolicy.AUTOMATIC,
    'MANUAL_BY_TOPIC': QoSLivelinessPolicy.MANUAL_BY_TOPIC,
}


class SubscriberQoS(Node):

    def __init__(self, qos_profile, node_name="publisher_qos_obj"):

        super().__init__(node_name)

        rclpy.logging.set_logger_level(
            node_name, rclpy.logging.LoggingSeverity.INFO)

        event_callbacks = SubscriptionEventCallbacks(
            incompatible_qos=self.incompatible_qos_clb,
            liveliness=self.liveliness_clb)

        self.subscriber = self.create_subscription(
            msg_type=String,
            topic='/qos_test',
            callback=self.listener_callback,
            qos_profile=qos_profile,
            event_callbacks=event_callbacks)

    def listener_callback(self, msg):
        self.get_logger().info("Data Received ="+str(msg.data))

    def incompatible_qos_clb(self, event):
        """
        This is the Callback that will be executed when the event of **Incompatible QoS** is
        triggered.
        """
        self.get_logger().error("SUBSCRIBER::: INCOMPATIBLE QoS Triggered!")
        self.get_logger().error(str(event.last_policy_kind))
        self.get_logger().error("############################")

    def liveliness_clb(self, event):
        """
        # https://docs.ros2.org/dashing/api/rmw/types_8h_source.html
        rmw_liveliness_changed_status_t
        Liveliness triggered
            int32_t alive_count;
            int32_t not_alive_count;
            int32_t alive_count_change;
            int32_t not_alive_count_change;
        """
        self.get_logger().error("SUBSCRIBER::: Liveliness Triggered !")
        self.get_logger().error("alive_count="+str(event.alive_count))
        self.get_logger().error("not_alive_count="+str(event.not_alive_count))
        self.get_logger().error("alive_count_change="+str(event.alive_count_change))
        self.get_logger().error("not_alive_count_change="+str(event.not_alive_count_change))
        self.get_logger().error("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")


def get_parser():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '-liveliness_lease_duration',
        type=int,
        default=2000,
        help='Select Policy for liveliness_lease_duration in milliseconds, use ros2 run qos_tests_pkg name_of_exe -liveliness_lease_duration 3000')

    parser.add_argument(
        '--policy', type=str, choices=POLICY_MAP.keys(), default='AUTOMATIC',
        help='The Liveliness policy type. AUTOMATIC|MANUAL_BY_TOPIC')

    return parser


def main(args=None):

    parser = get_parser()
    parsed_args = parser.parse_args()

    liveliness_lease_duration = Duration(
        seconds=parsed_args.liveliness_lease_duration / 1000.0)

    liveliness_policy = POLICY_MAP[parsed_args.policy]

    qos_profile_subscriber = QoSProfile(
        depth=1,
        liveliness=liveliness_policy,
        liveliness_lease_duration=liveliness_lease_duration)

    rclpy.init(args=args)
    sub_qos_obj = SubscriberQoS(qos_profile=qos_profile_subscriber,
                                node_name="subscriber_liveliness")

    rclpy.spin(sub_qos_obj)
    sub_qos_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    # ros2 run qos_tests_pkg subscriber_liveliness_exe -liveliness_lease_duration 450 --policy MANUAL_BY_TOPIC
    main()
```

> publisher_liveliness.py

```python
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos_event import PublisherEventCallbacks
from rclpy.qos import QoSProfile
from rclpy.qos import QoSLivelinessPolicy

from rclpy.duration import Duration

import time

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

POLICY_MAP = {
    'AUTOMATIC': QoSLivelinessPolicy.AUTOMATIC,
    'MANUAL_BY_TOPIC': QoSLivelinessPolicy.MANUAL_BY_TOPIC,
}


class PublisherQoS(Node):

    def __init__(self, qos_profile, publish_period, pub_topic_assert_period, publish_assert, node_name="publisher_qos_obj"):

        self.publish_assert = publish_assert

        self.group_timer_publisher = MutuallyExclusiveCallbackGroup()
        self.group_alive_timer = MutuallyExclusiveCallbackGroup()
        self.group_events_clb = MutuallyExclusiveCallbackGroup()

        super().__init__(node_name)

        rclpy.logging.set_logger_level(
            node_name, rclpy.logging.LoggingSeverity.INFO)

        event_callbacks = PublisherEventCallbacks(
            incompatible_qos=self.incompatible_qos_clb,
            liveliness=self.liveliness_clb)

        self.publisher_ = self.create_publisher(msg_type=String,
                                                topic='/qos_test',
                                                qos_profile=qos_profile,
                                                event_callbacks=event_callbacks,
                                                callback_group=self.group_events_clb)

        # Create a timer
        # Set the timer period to the asserted period because it is inside the Timer Callback
        self.publish_period = float(publish_period / 1000)
        self.pub_topic_assert_period = float(pub_topic_assert_period / 1000)
        self.swap_state_time = 5.0
        self.time_pause = 5.0
        self.counter = 0

        self.create_timer(self.publish_period, self.timer_callback,
                          callback_group=self.group_timer_publisher)

        self.create_timer(self.pub_topic_assert_period, self.alive_callback,
                          callback_group=self.group_alive_timer)

    def incompatible_qos_clb(self, event):
        """
        This is the Callback that will be executed when the event of **Incompatible QoS** is
        triggered.
        """
        self.get_logger().error("PUBLISHER::: INCOMPATIBLE QoS Triggered!")
        self.get_logger().error(str(event.last_policy_kind))
        self.get_logger().error("############################")

    def liveliness_clb(self, event):
        """
        Liveliness triggered
        """
        self.get_logger().error("PUBLISHER::: Liveliness Triggered!")
        self.get_logger().error(str(event.total_count_change))
        self.get_logger().error(str(event.total_count))
        self.get_logger().error("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")

    def publish_one_message(self):
        # Here you have the Callback method
        msg = String()
        test_time = self.get_clock().now()
        self.current_time_s, self.current_time_ns = test_time.seconds_nanoseconds()
        time_str = str(self.current_time_s)+","+str(self.current_time_ns)
        msg.data = time_str
        self.publisher_.publish(msg)
        self.get_logger().info('Published: "%s"' % msg)

    def timer_callback(self):
        # print a ROS2 log on the terminal with a great message!

        if self.counter > int(self.swap_state_time / self.publish_period):
            delta = 0.1
            range_steps = int(self.time_pause / delta)
            for i in range(range_steps):
                time.sleep(delta)
                self.get_logger().info("Paused ="+str(i*delta)+"/"+str(self.time_pause))
            self.counter = 0
        else:
            self.publish_one_message()
            self.counter += 1
            self.get_logger().info("Counter ="+str(self.counter))

    def alive_callback(self):
        self.i_am_alive()

    def i_am_alive(self):
        # https://docs.ros2.org/dashing/api/rclpy/api/topics.html
        # Publish that you are alive even if you do not publish any message in the pause phase
        if self.publish_assert:
            self.publisher_.assert_liveliness()
            self.get_logger().info("Publisher Alive...")


def get_parser():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '-liveliness_lease_duration',
        type=int,
        default=2000,
        help='Select Policy for liveliness_lease_duration in milliseconds, use ros2 run qos_tests_pkg name_of_exe -liveliness_lease_duration 3000')
    parser.add_argument(
        '-publish_period', type=int,
        help='How often you publish the message in publisher topic in milliseconds')
    parser.add_argument(
        '-topic_assert_period', type=int,
        help='How often (in positive integer milliseconds) the Talker will manually assert the '
             'liveliness of its Publisher.')
    parser.add_argument(
        '--publish_assert', type=str, choices=["yes", "no"], default="yes",
        help='If you want publish, assert manually. yes|no')
    parser.add_argument(
        '--policy', type=str, choices=POLICY_MAP.keys(), default='AUTOMATIC',
        help='The Liveliness policy type. AUTOMATIC|MANUAL_BY_TOPIC')

    return parser


def main(args=None):

    parser = get_parser()
    parsed_args = parser.parse_args()

    liveliness_lease_duration = Duration(
        seconds=parsed_args.liveliness_lease_duration / 1000.0)

    liveliness_policy = POLICY_MAP[parsed_args.policy]

    publish_period = parsed_args.publish_period

    topic_assert_period = parsed_args.topic_assert_period

    publish_assert_str = parsed_args.publish_assert
    publish_assert = (publish_assert_str == "yes")

    print("##########################"+str(publish_assert))

    qos_profile_publisher = QoSProfile(
        depth=1,
        liveliness=liveliness_policy,
        liveliness_lease_duration=liveliness_lease_duration)

    rclpy.init(args=args)
    pub_qos_obj = PublisherQoS(qos_profile=qos_profile_publisher,
                               publish_period=publish_period,
                               pub_topic_assert_period=topic_assert_period,
                               publish_assert=publish_assert,
                               node_name="publisher_liveliness_node")

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(pub_qos_obj)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        pub_qos_obj.destroy_node()

    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    # Command example : os2 run qos_tests_pkg publisher_liveliness_exe -liveliness_lease_duration 450 -publish_period 300 -topic_assert_period 1000 --publish_assert yes --policy MANUAL_BY_TOPIC
    main()
```
Precisamos comentar vários aspectos do código:

O fato de que agora você está usando grupos MutuallyExclusiveCallbackGroup

```python
self.group_timer_publisher = MutuallyExclusiveCallbackGroup()
self.group_alive_timer = MutuallyExclusiveCallbackGroup()
self.group_events_clb = MutuallyExclusiveCallbackGroup()
```

As razões são as seguintes:

* Como você executa apenas dois retornos de chamada regulares (timer_callback e alive_callback), se você usar RentrantCallbackGroups, o sistema executará várias vezes simultaneamente timer_callback ou alive_callback. Você não quer isso. Você deseja que esses retornos de chamada sejam executados apenas uma instância de cada um. É por isso que você usa mutuamente exclusivos.
O fato de que agora você está usando três grupos de retorno de chamada

```python
        event_callbacks = PublisherEventCallbacks(
            incompatible_qos=self.incompatible_qos_clb,
            liveliness=self.liveliness_clb)

        self.publisher_ = self.create_publisher(msg_type=String,
                                                topic='/qos_test',
                                                qos_profile=qos_profile,
                                                event_callbacks=event_callbacks,
                                                callback_group=self.group_events_clb)

        ...

        self.create_timer(self.publish_period, self.timer_callback,
                          callback_group=self.group_timer_publisher)

        self.create_timer(self.pub_topic_assert_period, self.alive_callback,
                          callback_group=self.group_alive_timer)
```
As razões são as seguintes:

* Você quer que pelo menos três Callbacks possam ser executados simultaneamente:
     * timer_callback
     * alive_callback
     * event_callbacks, que são o liveliness_clb ou o incompatível_qos_clb
O fato de que agora você está usando o executor Multithreaded

```python
executor = MultiThreadedExecutor(num_threads=3)
```
As razões são as seguintes:

* Você precisa de pelo menos três threads para processar três Callbacks em paralelo.

> Setup.py

```python
from setuptools import setup

package_name = 'qos_tests_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='duckfrost@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_liveliness_exe = qos_tests_pkg.publisher_liveliness:main',
            'subscriber_liveliness_exe = qos_tests_pkg.subscriber_liveliness:main',

        ],
    },
)
```

```bash
cd ~/ros2_ws/
colcon build --symlink-install --packages-select qos_tests_pkg
source install/setup.bash
```

## USANDO VIVIDADE AUTOMÁTICA
Você não publica a mensagem Alive; você publica a cada três segundos com a leaseDuration sendo dois segundos. Em teoria, isso deve acionar o erro de retorno de chamada de vivacidade. No entanto, do lado do assinante e do editor, nada acontece.

```bash
ros2 run qos_tests_pkg publisher_liveliness_exe -liveliness_lease_duration 2000 -publish_period 3000 -topic_assert_period 3000 --publish_assert no --policy AUTOMATIC
```
Veja o assinante:

Se você definir o LeaseDuration menor que o publicador, é uma QoS incompatível.
```bash
ros2 run qos_tests_pkg subscriber_liveliness_exe -liveliness_lease_duration 1000 --policy AUTOMATIC
```
No entanto, se você definir a vivacidade para um valor MAIOR do que o do editor, o Callback dispara do assinante para nos informar que você tem um editor ativo:

```bash
ros2 run qos_tests_pkg subscriber_liveliness_exe -liveliness_lease_duration 3000 --policy AUTOMATIC
```

> Output 

```bash
[ERROR] [1645025787.812470945] [subscriber_liveliness]: SUBSCRIBER::: Liveliness Triggered!
[ERROR] [1645025787.813001553] [subscriber_liveliness]: alive_count=1
[ERROR] [1645025787.813461276] [subscriber_liveliness]: not_alive_count=0
[ERROR] [1645025787.813915481] [subscriber_liveliness]: alive_count_change=1
[ERROR] [1645025787.814364720] [subscriber_liveliness]: not_alive_count_change=0
[ERROR] [1645025787.814814320] [subscriber_liveliness]: @@@@@@@@@@@@@@@@@@@@@@@@@@@@@
```

* alive_count_change=1, significa que você ganhou um editor.
* alive_count=1, agora você vê que tem um publicador Alive.

Tente parar o PUBLISHER agora enquanto o assinante ainda está ouvindo. Neste caso, agora o assinante está contando o tempo definido no leaseDuration do assinante. E então ele será acionado, informando que você perdeu um editor:

```bash
[ERROR] [1645025805.018105382] [subscriber_liveliness]: SUBSCRIBER::: Liveliness Triggered!
[ERROR] [1645025805.018746115] [subscriber_liveliness]: alive_count=0
[ERROR] [1645025805.019210407] [subscriber_liveliness]: not_alive_count=0
[ERROR] [1645025805.019749429] [subscriber_liveliness]: alive_count_change=-1
[ERROR] [1645025805.020284825] [subscriber_liveliness]: not_alive_count_change=0
[ERROR] [1645025805.020858597] [subscriber_liveliness]: @@@@@@@@@@@@@@@@@@@@@@@@@@@@@
```

* alive_count_change=-1, significa que você perdeu um editor.
* alive_count=0, agora você vê que não tem nenhum alive.
A conclusão é que:

* O valor LeaseDuration APENAS é usado ao usar o MANUAL_BY_TOPIC.
* Um dos casos em AUTOMATIC, onde o valor LeaseDuration é usado, é para uma verificação de compatibilidade de QoS.
* Outro caso em que você pode acionar o retorno de chamada de vivacidade é quando você fecha o nó do editor enquanto o assinante ainda está trabalhando.

Observe que o gatilho no assinante é instantâneo quando o editor morre. Portanto, a duração do arrendamento não é usada.

## USANDO O MÉTODO ALIVE para MANUAL_BY_TOPIC
Exemplo de um publicador rápido, mas uma mensagem Alive lenta
Neste caso, o tempo de publicação do sinal Alive é > que o LeaseDuration. Isso significa que deve dar um erro. O único momento em que isso não acontece é quando você publica porque a publicação tem prioridade sobre o sinal Alive.

Se eu posso falar com você, é óbvio que você está vivo. Não preciso de nenhum sinal especial para me dizer.

Neste exemplo, você vê que dá um gatilho de vivacidade quando o tópico publicado é pausado, porque:

* Você parou de publicar; portanto, você não recebe nenhuma mensagem.
* O sinal Alive é publicado a cada três segundos, o que é maior que os dois segundos da duração da concessão.

```bash
ros2 run qos_tests_pkg publisher_liveliness_exe -liveliness_lease_duration 2000 -publish_period 1000 -topic_assert_period 3000 --publish_assert yes --policy MANUAL_BY_TOPIC
```

```bash
ros2 run qos_tests_pkg subscriber_liveliness_exe -liveliness_lease_duration 3000 --policy AUTOMATIC
```

> Output 

```bash
[INFO] [1645026603.247400671] [subscriber_liveliness]: Data Received =1645026603,246501420
[INFO] [1645026604.247414913] [subscriber_liveliness]: Data Received =1645026604,246504289
[ERROR] [1645026607.247663506] [subscriber_liveliness]: SUBSCRIBER::: Liveliness Triggered!
[ERROR] [1645026607.248263356] [subscriber_liveliness]: alive_count=0
[ERROR] [1645026607.248822345] [subscriber_liveliness]: not_alive_count=1
[ERROR] [1645026607.249401287] [subscriber_liveliness]: alive_count_change=-1
[ERROR] [1645026607.250044846] [subscriber_liveliness]: not_alive_count_change=1
[ERROR] [1645026607.250583916] [subscriber_liveliness]: @@@@@@@@@@@@@@@@@@@@@@@@@@@@@
[ERROR] [1645026610.285465126] [subscriber_liveliness]: SUBSCRIBER::: Liveliness Triggered!
[ERROR] [1645026610.286062735] [subscriber_liveliness]: alive_count=1
[ERROR] [1645026610.286643833] [subscriber_liveliness]: not_alive_count=0
[ERROR] [1645026610.287334065] [subscriber_liveliness]: alive_count_change=1
[ERROR] [1645026610.287854927] [subscriber_liveliness]: not_alive_count_change=-1
[ERROR] [1645026610.288318913] [subscriber_liveliness]: @@@@@@@@@@@@@@@@@@@@@@@@@@@@@
[INFO] [1645026610.288989710] [subscriber_liveliness]: Data Received =1645026610,284628135
[INFO] [1645026611.247577412] [subscriber_liveliness]: Data Received =1645026611,246611642
```

Veja várias fases:

* Fase 1: o editor publica a cada segundo, então não há problema nisso.
* Fase 2: A editora entra na fase de pausa, onde não publicará até 5 segundos depois; portanto, o único que poderia salvá-lo é o método Alive. No entanto, o método Alive publica a cada três segundos. Portanto, após três segundos (indicados no LeaseDuration do assinante), o gatilho mostra que agora você tem um NOT_ALIVE_COUNT:

```bash
* not_alive_count=1
* alive_count_change=-1
* not_alive_count_change=1
```
Fase 3: O editor começa a publicar novamente e, portanto, aciona o Callback novamente, informando que você recuperou esse editor:

```bash
* alive_count=1
* not_alive_count=0
* alive_count_change=1
* not_alive_count_change=-1
```

### Exemplo de um publicador lento, mas uma mensagem ativa rápida
Nesse caso, embora o editor seja mais lento do que o leaseDuration necessário (4 segundos para publicar, LeaseDuration 2 segundos) porque o sinal Alive é publicado em um período de 1 segundo, não importa se o editor é lento. Mesmo quando você pausa o editor por 5 segundos, isso não importa porque o sinal Alive está sempre publicando.

```bash
ros2 run qos_tests_pkg publisher_liveliness_exe -liveliness_lease_duration 2000 -publish_period 4000 -topic_assert_period 1000 --publish_assert yes --policy MANUAL_BY_TOPIC
```

```bash
ros2 run qos_tests_pkg subscriber_liveliness_exe -liveliness_lease_duration 3000 --policy AUTOMATIC
```

Aqui, nenhum problema ocorrerá, apenas a primeira mensagem informando que há um editor. E se você parar o editor, ele informará após a duração do aluguel que não há mais editores:

> Output

```bash
[ERROR] [1645027798.044206747] [subscriber_liveliness]: SUBSCRIBER::: Liveliness Triggered!
[ERROR] [1645027798.044944871] [subscriber_liveliness]: alive_count=1
[ERROR] [1645027798.045553071] [subscriber_liveliness]: not_alive_count=0
[ERROR] [1645027798.046156011] [subscriber_liveliness]: alive_count_change=1
[ERROR] [1645027798.046756773] [subscriber_liveliness]: not_alive_count_change=0
[ERROR] [1645027798.047374311] [subscriber_liveliness]: @@@@@@@@@@@@@@@@@@@@@@@@@@@@@
```

E se você parar o editor, ele informará após a duração do aluguel que não há mais editores:

```bash
[ERROR] [1645027892.103160561] [subscriber_liveliness]: SUBSCRIBER::: Liveliness Triggered!
[ERROR] [1645027892.103815172] [subscriber_liveliness]: alive_count=0
[ERROR] [1645027892.104451484] [subscriber_liveliness]: not_alive_count=0
[ERROR] [1645027892.105099979] [subscriber_liveliness]: alive_count_change=-1
[ERROR] [1645027892.105572328] [subscriber_liveliness]: not_alive_count_change=0
[ERROR] [1645027892.106211099] [subscriber_liveliness]: @@@@@@@@@@@@@@@@@@@@@@@@@@@@@
```

### Chamadas de retorno do evento:
Você usou retornos de chamada de evento para capturar os diferentes problemas de QoS.

Se você quiser ver todas as opções, dê uma olhada no código-fonte aqui: [qos_event](https://github.com/ros2/rclpy/blob/master/rclpy/rclpy/qos_event.py)

## QoS em Bolsas ROS2
Existem algumas situações em que você deseja substituir o QoS do ROS2Bags. Talvez porque o perfil gravado originalmente seja muito rígido ou você queira simular um sistema mais rígido com os dados de reprodução.

No entanto, a gravação tem suas ressalvas.

Gravando ROSbags QoS
Rosbags armazena o QoS original.

```bash
ros2 topic info /scan --verbose
```

> Output 

```bash
Publisher count: 1

Node name: lidar_1
Node namespace: /
Topic type: sensor_msgs/msg/LaserScan
Endpoint type: PUBLISHER
GID: 2e.cc.10.01.fd.63.45.1e.07.68.af.91.00.00.3e.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  Durability: VOLATILE
  Lifespan: 9223372036854775807 nanoseconds
  Deadline: 9223372036854775807 nanoseconds
  Liveliness: AUTOMATIC
  Liveliness lease duration: 9223372036854775807 nanoseconds

Subscription count: 0
```

Grave o tópico de verificação em um rosbag:

```bash
cd ~/ros2_ws/src
mkdir rosbags
cd rosbags
ros2 bag record /scan -o neobotixs_mp400_scan_bag
```

HIT CTRL+C quando terminar

Agora, repita o loop e veja se o QoS foi registrado corretamente:

Como o robô ainda está publicando o tópico `/scan`, você remapeará o tópico `rosbags/scan` para `/scan2`.

```bash
cd ~/ros2_ws/src
cd rosbags
ros2 bag play --remap /scan:=/scan2 -l neobotixs_mp400_scan_bag
```

```bash
ros2 topic info /scan2 --verbose
```

> Output 

```bash
Type: sensor_msgs/msg/LaserScan

Publisher count: 1

Node name: rosbag2_player
Node namespace: /
Topic type: sensor_msgs/msg/LaserScan
Endpoint type: PUBLISHER
GID: 60.84.10.01.d3.55.06.a0.06.e2.5e.0a.00.00.15.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  Durability: VOLATILE
  Lifespan: 9223372036854775807 nanoseconds
  Deadline: 9223372036854775807 nanoseconds
  Liveliness: AUTOMATIC
  Liveliness lease duration: 9223372036854775807 nanoseconds

Subscription count: 0
```

### Repetir Alterando a QoS
**1. Crie um arquivo de configuração de QoS** chamado qos_override.yaml

```bash
cd ~/ros2_ws/src/rosbags/
touch qos_override.yaml
```

Aqui, altere o seguinte:

* confiabilidade -> CONFIÁVEL
* histórico -> KEEP_ALL
* deadline -> 1 segundo e 100 nanossegundos

```yaml
# qos_override.yaml
/scan:
  reliability: reliable
  history: keep_all
  deadline:
    sec: 1
    nsec: 100
```

**2. Reproduza o ROSbag** com o arquivo QoS personalizado qos_override.yaml

Como o robô ainda está publicando o tópico `/scan`, você remapeará o tópico `rosbags/scan` para `/scan2`.

```bash
cd ~/ros2_ws/src/rosbags/
ros2 bag play --remap /scan:=/scan2 --qos-profile-overrides-path qos_override.yaml -l neobotixs_mp400_scan_bag
```

Agora o tópico `/scan2` aparece:

```bash
ros2 topic info /scan2 --verbose
```

> Output 

```bash
Type: sensor_msgs/msg/LaserScan

Publisher count: 1

Node name: rosbag2_player
Node namespace: /
Topic type: sensor_msgs/msg/LaserScan
Endpoint type: PUBLISHER
GID: 79.3e.10.01.7a.6e.af.7a.7b.f5.23.31.00.00.15.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  Durability: VOLATILE
  Lifespan: 9223372036854775807 nanoseconds
  Deadline: 1000000100 nanoseconds
  Liveliness: AUTOMATIC
  Liveliness lease duration: 9223372036854775807 nanoseconds

Subscription count: 0
```

Você também pode verificar isso no RVIZ2.

* Adicione os dois lasers e defina seu tópico QoS corretamente para visualizar ambos.
* Você deve ver algo assim, onde o verde é `scan2` e o vermelho é `scan1`.

<div align="center">
     <img src="./img/output9.png" alt="Particles Filter" width="600px">
</div>