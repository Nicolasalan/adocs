# Mapping
To carry out autonomous navigation of a robot, it is necessary to have a map of the environment in which it must move. This map can be created by the robot itself from the readings of its sensors, such as the laser. This process of creating the map is known as mapping in ROS Navigation.

* Explicação de um mapa e por que você precisa dele
* O que você precisa para construir um mapa no ROS2
* Como usar o cartógrafo para construir um mapa
* Como fornecer o mapa construído para outros aplicativos ROS2
* Um novo conceito de navegação: Navigation Lifecycle Manager

Um mapa no ROS é uma representação do ambiente em que o robô está operando, utilizado para localização e planejamento de trajetórias. Em termos simples, é uma grade de ocupação onde valores específicos em cada célula do mapa representam obstáculos.

Para construir um mapa, é necessário que o robô esteja equipado com LIDAR, odometria, bateria e um ambiente. O processo de criação de mapas e localização simultânea é conhecido como SLAM, e existem diversos algoritmos SLAM disponíveis para ROS2, como o `Cartógrafo` e o `SLAM-Toolbox`.

O Cartógrafo é um sistema que fornece localização e mapeamento simultâneos em tempo real em 2D e 3D em diversas configurações de sensores e plataformas. O `Cartographer_ros` é um wrapper ROS do Cartógrafo para que ele possa ser integrado ao ROS.

Para criar um arquivo de inicialização do Cartógrafo para o seu robô, é necessário lançar dois nós: o `cartographer_node` e o `cartographer_occupancy_grid_node`. É preciso indicar o pacote, o executável e os parâmetros necessários, como o diretório de configuração e o nome do arquivo de configuração. O arquivo de inicialização permite iniciar vários nós a partir de um único arquivo e definir parâmetros específicos para cada nó.

> O pacote Cartógrafo pode ser comparado a um cartógrafo humano, que utiliza instrumentos para mapear e desenhar a topografia de um terreno desconhecido. Da mesma forma, o Cartógrafo utiliza sensores e algoritmos para criar um mapa detalhado de um ambiente desconhecido em tempo real. Assim como um cartógrafo humano precisa ter conhecimento técnico e habilidades para criar mapas precisos, o pacote Cartógrafo requer configuração e ajustes para fornecer resultados confiáveis e precisos.

### Diferença entre Cartógrafo e Gmapping

O Cartographer ROS e o GMapping são dois pacotes de software de mapeamento simultâneo (SLAM) amplamente utilizados em robótica. Eles possuem algumas diferenças significativas em termos de funcionalidade e recursos.

O Cartographer ROS é um sistema de mapeamento simultâneo baseado em laser que pode ser usado para criar mapas 2D e 3D de ambientes desconhecidos em tempo real. Ele utiliza algoritmos avançados de odometria visual e sensorial, e sua saída é um mapa altamente preciso e detalhado. O Cartographer é projetado para ser altamente escalável, permitindo a utilização em uma ampla variedade de aplicações.

Já o GMapping é um pacote de mapeamento SLAM baseado em grade que também é amplamente utilizado em robótica. Ele usa dados do sensor de varredura a laser para criar mapas de ambientes desconhecidos em tempo real. O GMapping é conhecido por ser rápido e robusto, e pode ser usado em uma ampla variedade de robôs móveis.

Em termos de vantagens e desvantagens, o Cartographer ROS é conhecido por sua precisão e capacidade de mapear ambientes complexos em tempo real. No entanto, ele é um pouco mais exigente em termos de hardware do que o GMapping e pode ser mais difícil de configurar. O GMapping, por outro lado, é mais fácil de usar e requer menos recursos de hardware, mas pode não ser tão preciso em ambientes complexos quanto o Cartographer.

Em resumo, o Cartographer ROS é ideal para aplicações que exigem alta precisão de mapeamento em ambientes complexos, enquanto o GMapping é uma ótima opção para aplicações que requerem um mapeamento rápido e simples em ambientes mais simples.

### Para iniciar o `cartógrafo`, inicie dois nós:

**1. Lançamento do cartógrafo_node**

Estes são os campos que precisa indicar no lançamento do nó:

* O `cartographer_node` é fornecido pelo pacote `cartographer_ros`.
* O executável é chamado `cartographer_node`.
* Os parâmetros necessários são:
     * `use_sim_time`: é um booleano que indica se o nó deve sincronizar seu tempo com a simulação
* Os argumentos são:
     * `configuration_directory`: o diretório onde encontrar os arquivos de configuração
     * `configuration_basename`: o nome do arquivo de configuração

```python
     package='cartographer_ros', 
     executable='cartographer_node', 
     name='cartographer_node',
     output='screen',
     parameters=[{'use_sim_time': True}],
     arguments=['-configuration_directory', cartographer_config_dir,
               '-configuration_basename', configuration_basename]
```

**2. Lançamento do occupation_grid_node**

Estes são os campos que precisa indicar no lançamento do nó:

* O `occupancy_grid_node` é fornecido pelo pacote `cartographer_ros`.
* O executável é chamado `occupancy_grid_node`.
* Os parâmetros necessários são:
     * `use_sim_time`: é um booleano que indica se o nó deve sincronizar seu tempo com a simulação
* Os argumentos são:
     * `resolution`: número de metros por grade no mapa
     * `publish_period_sec`: com que frequência (em número de segundos) o mapa é publicado no tópico `/map`.

```python
     package='cartographer_ros',
     executable='occupancy_grid_node',
     output='screen',
     name='occupancy_grid_node',
     parameters=[{'use_sim_time': True}],
     arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
```

