# Conceitos Básicos
Como estruturar e iniciar programas ROS2 (pacotes e arquivos de inicialização)
Como criar programas ROS2 básicos (baseados em Python)
Conceitos básicos do ROS2: nós, bibliotecas de cliente, etc.

## O que é ROS2?
É possível que você tenha feito este curso para responder a esta pergunta: "O que é ROS2?"

Por enquanto, no entanto, será mais útil experimentar o que o ROS2 pode fazer.

### 2.1.1 Mover um Robô com ROS2
No canto direito da tela, você tem seu primeiro robô simulado: o TurtleBot3. Vamos mover esse robô agora!

Como você move o TurtleBot?

O método mais simples é controlar o robô usando um programa ROS2 existente. Em seguida, os executáveis criados durante a compilação de um programa ROS2 são usados para executá-lo. Mais adiante neste guia, você aprenderá mais sobre compilação.

Como ele já existe neste espaço de trabalho, você iniciará um programa ROS2 (executável) feito anteriormente que permite mover o robô usando o teclado.

Antes de tentar executar o executável, você precisará fazer algum trabalho preliminar. Não pense nos significados dos comandos abaixo; você aprenderá como usá-los durante este tutorial.

Então, continue com isso. Para obter seu espaço de trabalho, execute os seguintes comandos no shell:
```bash
source /opt/ros/humble/setup.bash
source /home/simulations/ros2_sims_ws/install/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Agora, você pode usar as teclas indicadas no shell Output para mover o robô.

> LEMBRE-SE, você precisa se concentrar no terminal (Shell) onde você lançou o programa para que as chaves tenham efeito. Você sabe que focou corretamente quando o cursor começa a piscar.

Quando você cansar de brincar com o robô, pode apertar Ctrl+C para interromper a execução do programa (lembre-se de ter o foco). Você pode fechar o Webshell #1 para continuar o curso, se desejar.

Dê uma olhada no que você aprendeu até agora. A palavra-chave ros2 é usada para todos os comandos ROS2. Portanto, para iniciar programas, você terá duas opções:

Inicie o programa ROS2 executando diretamente o arquivo executável.
Inicie o programa ROS2 iniciando um arquivo de inicialização.
Pode parecer mais fácil usar o comando run para iniciar executáveis, mas você entenderá mais tarde por que o comando launch também é útil.

Por enquanto, você pode executar diretamente o arquivo executável. A estrutura do comando é a seguinte:

```bash
ros2 run <package_name> <executable_file>
```

Como você pode ver, o comando possui dois parâmetros: o primeiro parâmetro é o nome do pacote que contém o arquivo executável. O segundo parâmetro é o nome do arquivo executável (que é armazenado dentro do pacote).

Para usar um arquivo de inicialização, a estrutura do comando seria a seguinte:
```bash
ros2 launch <package_name> <launch_file>
```
Como você pode ver, este comando também possui dois parâmetros: o primeiro parâmetro é o nome do pacote que contém o arquivo de inicialização. O segundo parâmetro é o nome do arquivo de inicialização (que é armazenado no pacote).
