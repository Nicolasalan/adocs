# Introdução

A navegação é uma habilidade fundamental para os robôs. Ela permite que eles se movam de um ponto a outro e evitem obstáculos. No entanto, como um robô sabe onde está, para onde deve ir e como evitar colidir com qualquer coisa em seu caminho?

Neste documento, discutiremos o processo de navegação dos robôs, que é semelhante ao dos humanos. Para se mover de um ponto a outro, o robô precisa realizar os seguintes passos:

* `Mapeamento`: O robô precisa conhecer o local em que está e construir um mapa dele.

* `Localização`: Em seguida, ele precisa saber onde está no ambiente em relação ao mapa criado.

* `Planejamento de caminho`: O robô precisa planejar a melhor rota para se mover entre dois pontos, levando em consideração o mapa criado e sua posição atual.

* `Controle do robô e prevenção de obstáculos`: Por fim, o robô precisa enviar mensagens para as rodas ou outros dispositivos de movimento para seguir o caminho planejado, evitando obstáculos no caminho.

Todos esses processos são complexos para construir do zero. É aí que o ROS (Robot Operating System) ajuda a construir robôs de forma mais eficiente. O ROS possui pacotes pré-construídos para navegação, permitindo que os desenvolvedores se concentrem em personalizar e otimizar a navegação para o seu robô específico.

### 1.1 Experiência prática:

Para começar a usar a navegação no ROS, é necessário baixar o repositório correspondente e os arquivos necessários. Isso pode ser feito através de comandos específicos executados em um terminal de linha de comando.

```bash
cd ~/ws/src
git clone https://bitbucket.org/theconstructcore/ros2_nav_files.git
```
Isso fará o download dos arquivos para o espaço de trabalho. Em seguida, obterá a distribuição do ROS2 que deseja usar e compilará o espaço de trabalho.

```bash
source /opt/ros/humble/setup.bash
cd ~/ws
colcon build
source install/setup.bash
```
Para executar o arquivo de navegação, precisará executar o seguinte comando:

```bash
ros2 launch nav2_course nav2_demo.launch.py
```
Para iniciar o sistema de navegação do robô, é necessário fornecer uma localização inicial. Para isso, é preciso selecionar o botão `2D Pose Estimate` no RVIZ e, em seguida, clicar no mapa na posição e orientação desejada para o robô na simulação.

Após alguns segundos, o mapa RVIZ mudará, mostrando a localização do robô e os mapas de custos de navegação. 

Depois de localizar o robô, pode fornecer um objetivo para que ele possa se mover de forma autônoma. Clique no seguinte botão no RVIZ: `2D Nav Goal` e, em seguida, clique no mapa na posição desejada para o robô.

Agora pode movimentar seu robô na simulação enviando-lhe metas através do RVIZ!

### Ferramentas de navegação ROS2
* Carregue, sirva e armazene mapas de ambiente (`Map Server`)
* Localize o robô no mapa (`AMCL`)
* Planeje um caminho de A a B contornando obstáculos (`Nav2 Planner`)
* Controle o robô conforme ele segue o caminho (`Nav2 Controller`)
* Converta os dados do sensor em uma representação do mundo com reconhecimento de obstáculos (`Nav2 Costmap 2D`)
* Compute comportamentos de recuperação em caso de falha (`Nav2 Recoveries`)
* Gerenciar o ciclo de vida dos servidores (`Nav2 Lifecycle Manager`)
* plug-ins para habilitar seus próprios algoritmos e comportamentos personalizados (`Nav2 BT Server`)