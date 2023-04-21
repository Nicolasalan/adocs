# Plugins de navegação e criação de plug-ins personalizados

- A estrutura básica e os plugins usados por padrão no Navigation2 (Nav2)
- Como criar seu plug-in Costmap personalizado
- Como criar seu plug-in personalizado do Planner
- Como criar seu plug-in de controlador personalizado

## Plug-ins no Nav2

### Introdução

Por que você usa PLUGINS no Nav2?
Os plug-ins são usados porque melhoram a flexibilidade do pipeline Nav2. O uso de plug-ins permite que você altere apenas um arquivo .yaml do controlador, planejador etc. e altere completamente a funcionalidade sem muita sobrecarga de compilação.

É especialmente útil para Costmaps porque os filtros Costmap empilham uns sobre os outros, permitindo que os plug-ins alterem a maneira como os Costmaps são processados para navegação rapidamente.

O Nav2 tem uma grande seleção de plug-ins PLUG and PLAY. Esta é outra grande vantagem. Os desenvolvedores podem criar plugins seguindo uma API básica. Estarão prontos para uso na aplicação desejada, com a segurança de que irão se conectar aos sistemas de navegação necessários.

Aqui está um link para a lista de plugins atualmente disponíveis: [Nav2 PLUGINS](https://navigation.ros.org/plugins/index.html?highlight=plugins).

A criação de NOVOS PLUGINS e o upload deles para o REPO geral também são incentivados. Isso enriquecerá a pilha do Nav2 e ajudará todos a terem melhores sistemas e aplicativos de navegação.

### Plug-ins padrão
Aqui você mostrará os plug-ins padrão comumente usados para navegação. Também comentaremos alguns trechos de código padrão para todos os plugins.

Primeiro, onde você define o plugin que deseja usar?
A resposta está nos arquivos .yaml que carregam os parâmetros para seus diferentes nós de navegação.

#### Mapas de custo
No arquivo `planner_server.yaml`, você pode ver a seguinte seção de parâmetros:
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_footprint
      use_sim_time: True
      robot_radius: 0.3
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True
      obstacle_layer:
          plugin: "nav2_costmap_2d::ObstacleLayer"
          enabled: True
          observation_sources: scan
          scan:
            topic: /scan
            max_obstacle_height: 2.0
            clearing: True
            marking: True
            data_type: "LaserScan"
            raytrace_max_range: 3.0
            raytrace_min_range: 0.0
            obstacle_max_range: 2.5
            obstacle_min_range: 0.0
```
Dá uma olhada nessa linha:
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
        ...
        plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
        ...
```
Dentro do namespace `global_costmap/global_costmap/ros__parameters`, você está carregando uma lista dentro dos plugins de parâmetro com vários nomes.

Esses nomes podem ser o que você quiser porque correspondem ao namespace que você definiu posteriormente, assim:
```yaml
static_layer:
    ...PARAMETERS
inflation_layer:
    ...PARAMETERS
obstacle_layer:
    ...PARAMETERS
```

Neste caso, você tem três plugins nomeados:

* `static_layer`, que carrega o plugin plugin: "nav2_costmap_2d::StaticLayer", [DEFINIÇÃO](https://github.com/ros-planning/navigation2/blob/main/nav2_costmap_2d/plugins/static_layer.cpp)
* `inflation_layer`, que carrega o plug-in plugin: "nav2_costmap_2d::InflationLayer", [DEFINIÇÃO](https://github.com/ros-planning/navigation2/blob/main/nav2_costmap_2d/plugins/inflation_layer.cpp)
* `obstacle_layer`, que carrega o plug-in plugin: "nav2_costmap_2d::ObstacleLayer", [DEFINIÇÃO](https://github.com/ros-planning/navigation2/blob/main/nav2_costmap_2d/plugins/obstacle_layer.cpp)

Todos esses plugins estão na lista fornecida aqui: [Nav2 PLUGINS](https://navigation.ros.org/plugins/index.html?highlight=plugins).

Vá para o código onde esses plugins estão definidos. Você verá que eles possuem estruturas muito semelhantes, principalmente quando regulam o mesmo elemento como `Costmaps`, `Planning`, `Controller` ou `Behavior Trees`.

Aqui você tem uma versão simplificada para o plug-in `Static Layer`:
```cpp
...INCLUDES...


PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::StaticLayer, nav2_costmap_2d::Layer)


namespace nav2_costmap_2d
{

StaticLayer::StaticLayer()
: map_buffer_(nullptr)
{
}

StaticLayer::~StaticLayer()
{
}

void
StaticLayer::onInitialize()
{
  ...CODE...
}

void
StaticLayer::activate()
{
    ...CODE...
}

void
StaticLayer::deactivate()
{
  ...CODE...
}

void
StaticLayer::reset()
{
  ...CODE...
}

void
StaticLayer::getParameters()
{
  ...CODE...
}

void
StaticLayer::processMap(const nav_msgs::msg::OccupancyGrid & new_map)
{
  ...CODE...
}

void
StaticLayer::matchSize()
{
  ...CODE...
}

unsigned char
StaticLayer::interpretValue(unsigned char value)
{
  ...CODE...
}

void
StaticLayer::incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map)
{
  ...CODE...
}

void
StaticLayer::incomingUpdate(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update)
{
  ...CODE...
}


void
StaticLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  ...CODE...
}

void
StaticLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  ...CODE...
}

/**
  * @brief Callback executed when a parameter change is detected
  * @param event ParameterEvent message
  */
rcl_interfaces::msg::SetParametersResult
StaticLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  ...CODE...
}

}  // namespace nav2_costmap_2d
```
Como você pode ver, é um monte de métodos. Estamos mostrando isso porque existem dois tipos de métodos aqui:

* Métodos padrão são usados por seu código exclusivo porque você deve calcular algo, acessar um tópico ou um banco de dados e executar um algoritmo de aprendizado profundo - tudo o que você precisa para executar sua tarefa.

* Métodos do `API PLUGIN`: Você os substituirá da classe pai, neste caso, `nav2_costmap_2d::Layer`. Cada um tem seus métodos OBRIGATÓRIOS e OPCIONAIS. Esses métodos permitem que os plug-ins funcionem plug and play porque o sistema de carregamento de plug-ins chama esses métodos para executar as funções essenciais do plug-in para a tarefa. Aqui você tem a lista. Como você pode ver, todos eles são virtuais para permitir esta funcionalidade:
```cpp
virtual void onInitialize();

virtual void activate();

virtual void deactivate();

virtual void reset();

virtual bool isClearable() {return false;}

virtual void updateBounds(
double robot_x, double robot_y, double robot_yaw, double * min_x,
double * min_y, double * max_x, double * max_y);

virtual void updateCosts(
nav2_costmap_2d::Costmap2D & master_grid,
int min_i, int min_j, int max_i, int max_j);

virtual void matchSize();
```
#### Planejador
A mesma coisa acontece com o planejador. Dentro do arquivo `planner_server.yaml`, você encontra os seguintes parâmetros de carregamento:
```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```
Veja aqui você tem a linha de código:
```yaml
planner_plugins: ["GridBased"]  
```
Após alguns testes, parece que **NÃO PODE** alterar o **NOME** da **TAG do PLUGIN**. Portanto, deve ser **GridBased** como o nome. Caso contrário, você receberá um erro ao executar o planejador:
```bash
planner GridBased is not a valid planner. Planner names are: WHATEVER_NAME_YOU_GAVE.
```
Neste caso, ele está carregando o `nav2_navfn_planner` [CODE](https://github.com/ros-planning/navigation2/tree/main/nav2_navfn_planner) do `plug-in nav2_navfn_planner/NavfnPlanner`.
```cpp
NavfnPlanner();

~NavfnPlanner();


void configure(
const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;


void cleanup() override;


void activate() override;

void deactivate() override;

nav_msgs::msg::Path createPlan(
const geometry_msgs::msg::PoseStamped & start,
const geometry_msgs::msg::PoseStamped & goal) override;
```
Você não o está definindo virtualmente, apenas configurando-o para substituir.

Você usa virtual para a declaração de função de classe base e, se quiser a sua própria, tem a opção de ser substituído. É necessário apenas para o método base.

Você usa override para uma substituição de classe derivada. Não é obrigatório, mas dá erro de compilação caso os métodos não coincidam, o que ajuda a manter o código.

Nesse caso de código, o desenvolvedor queria verificar se os métodos coincidiam com a classe base. Nenhum virtual foi usado.
#### Controlador
Nesse caso, você pode encontrar o carregamento do parâmetro do plug-in no arquivo `controller.yaml`, assim:
```yaml
controller_server:
  ros__parameters:
    ...
    controller_plugins: ["FollowPath"]

    ...
    FollowPath:
        plugin: "dwb_core::DWBLocalPlanner"
        ..NEEDED PLUGIN PARAMETERS
        critics: ["RotateToGoal", "Oscillation", "ObstacleFootprint", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
```
Aqui você usa o nome da tag de plug-in FollowPath e carrega o plug-in do `dwb_core::DWBLocalPlanner`. Seu código está AQUI.

Observe que você também está carregando dentro dele MAIS plugins, neste caso, críticos:
```cpp
critics: ["RotateToGoal", "Oscillation", "ObstacleFootprint", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
```
Esses plugins são definidos [AQUI](https://github.com/ros-planning/navigation2/tree/main/nav2_dwb_controller/dwb_critics). Por exemplo, aqui está o plug-in [OSCILLATION CRITIC](https://github.com/ros-planning/navigation2/blob/main/nav2_dwb_controller/dwb_critics/src/oscillation.cpp). Os plugins críticos são da classe base `dwb_core::TrajectoryCritic`.

Agora você criará vários exemplos e exercícios que ensinam como modificar e criar Plugins CUSTOM para **Costmaps**, **Planning** e **Controllers**.

## Criação de plug-ins Nav2 personalizados

A maioria das etapas para criar os plug-ins são idênticas, mas existem alguns elementos diferentes baseados essencialmente na classe Base na qual cada plug-in pode ser baseado.

Você também precisa de uma pilha de navegação funcional para testar todos os novos plugins. Para isso, configure uma navegação padrão pronta para uso.

### Configuração de navegação

Se você deseja que a localização comece corretamente e não precise mover o robô para relocalizar, você tem duas opções:

1. Inicie a localização desde o início.
2. Reinicie a simulação sempre que reiniciar a localização.

Baixe em seu ~/ros2_ws/src o seguinte código:

```bash
cd ~/ros2_ws/src/
git clone https://bitbucket.org/theconstructcore/neobotix_mp_400_navigation.git
cd ~/ros2_ws
colcon build
```
Aqui você encontrará tudo o que precisa para começar a navegar. Então experimente e execute os seguintes comandos:

**Iniciar localização**
```bash
cd ~/ros2_ws
source install/setup.bash;reset;ros2 launch localization_server localization.launch.py
```
**Iniciar planejamento do caminho**
```bash
cd ~/ros2_ws
source install/setup.bash;reset;ros2 launch path_planner_server pathplanner.launch.py
```
Agora você deve acessar as Ferramentas Gráficas (deve abrir automaticamente) e alterar o arquivo de configuração do RVIZ para o planejamento do caminho. Deve estar no caminho `~/ros2_ws/src/path_planner_server/rviz_config/pathplanning.rviz`.

Vá para `File` -> `Open Config` e selecione esse arquivo.

Agora você deve fazer aparecer os Costmaps, e você pode definir uma meta, e deve gerar um caminho e segui-lo, tanto no RVIZ quanto na simulação assim:

### Plug-in do mapa de custos

O objetivo deste primeiro exemplo é ensinar o básico de como criar seu próprio plugin. Mais precisamente, seu próprio plug-in para filtragem de mapas de custo. Siga os passos para criá-lo:

#### Etapa 1: Criar o pacote
Crie um pacote para armazenar todos os plugins para Costmaps. Execute os seguintes comandos:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake custom_nav2_costmap_plugin --dependencies rclcpp nav2_costmap_2d pluginlib
cd ~/ros2_ws
colcon build --packages-select custom_nav2_costmap_plugin
source install/setup.bash
```
#### Etapa 2: criar o arquivo de origem e o arquivo de cabeçalho
Crie os arquivos `.cpp` e `.hpp` que definem o plugin:
```bash
cd ~/ros2_ws/src/custom_nav2_costmap_plugin
touch src/gradient_layer.cpp
touch include/custom_nav2_costmap_plugin/gradient_layer.hpp
```
> gradient_layer.hpp
```cpp
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions, and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions, and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING BUT NOT
 *  LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/
#ifndef GRADIENT_LAYER_HPP_
#define GRADIENT_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace custom_nav2_costmap_plugin
{

class GradientLayer : public nav2_costmap_2d::Layer
{
public:
  GradientLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  virtual void onFootprintChanged();

  virtual bool isClearable() {return false;}

private:
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  // Indicates that the entire gradient should be recalculated next time.
  bool need_recalculation_;

  // Size of gradient in cells
  int GRADIENT_SIZE = 20;
  // Step of increasing cost per one cell in gradient
  int GRADIENT_FACTOR = 10;
};

}  // namespace custom_nav2_costmap_plugin

#endif  // GRADIENT_LAYER_HPP_
```

Primeiro, vamos comentar sobre o básico do arquivo `*.hpp`:

Primeiro, defina um namespace para a classe do seu plug-in. Isso evita que o sistema se confunda com métodos que possuem o mesmo nome de outros plugins. No seu caso, o namespace que você usa é `custom_nav2_costmap_plugin`.

```cpp
namespace custom_nav2_costmap_plugin
{
   ..DEFINIÇÃO DE CLASSE PERSONALIZADA
}
```
O nome de sua classe personalizada é `GradientLayer` e herda da classe base chamada `nav2_costmap_2d::Layer`. É daqui que você herda métodos e os sobrescreve.
```cpp
classe GradientLayer : public nav2_costmap_2d::Layer
{
         ... MÉTODOS DE CLASSE E VARIÁVEIS
};
```
Todos os métodos que você substituirá da classe base estão dentro da seção pública de sua classe:
```cpp
public:
  GradientLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  virtual void onFootprintChanged();

  virtual bool isClearable() {return false;}
```

* `updateBounds()`: OBRIGATÓRIO. Sendo este método obrigatório, você PRECISA DEFINI-LO AQUI no seu plugin. Este é o responsável por decidir quanto do Costmap é atualizado com base nos limites do robô, posição, etc. Quanto menos área, mais rápido o seu plugin.

* `updateCosts()`: OBRIGATÓRIO. Ele atualiza os custos dentro dos limites definidos no método updateBlounds(). Isso pode ser feito adicionando no TOPO os custos já existentes calculados pelos plug-ins anteriores em execução ou você pode sobrescrever os valores anteriores completamente.

* `reset()`: OBRIGATÓRIO. Quando o sistema de plug-in de navegação reinicia o plug-in, por exemplo, por meio das árvores de comportamento, esse é o método executado.

* `onInitialize()`: NÃO É NECESSÁRIO. Coloque aqui qualquer código que você precisa executar ao iniciar o plug-in, como buscar parâmetros, inicializar variáveis e contadores.

* `matchSize()`: NÃO OBRIGATÓRIO. Você o chama quando o tamanho do mapa é alterado.

* `onFootprintChanged()`: NÃO É NECESSÁRIO. Isso é útil se você tiver um robô que pode alterar as pegadas, como conversíveis, anexar ferramentas ou carregar objetos maiores que a pegada original. Execute aqui o que você precisar fazer quando isso acontecer.

Como você pode ver no exemplo, aqui você não definiu `matchSize()`, mas como NÃO É NECESSÁRIO, ele usará o método padrão (que não faz nada, pois não é implementado na classe base CPP [layer.hpp](https://github.com/ros-planning/navigation2/blob/galactic/nav2_costmap_2d/include/nav2_costmap_2d/layer.hpp), [layer.cpp](https://github.com/ros-planning/navigation2/blob/galactic/nav2_costmap_2d/src/layer.cpp) ).

Agora, veja o código para o código `.cpp`:

> gradient_layer.cpp
```cpp
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions, and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions, and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING BUT NOT
 *  LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/
#include "custom_nav2_costmap_plugin/gradient_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace custom_nav2_costmap_plugin
{

GradientLayer::GradientLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
GradientLayer::onInitialize()
{
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  need_recalculation_ = false;
  current_ = true;
}

// The method is called to ask the plugin: which area of Costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
GradientLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;

    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}

// The method is called when footprint was changed.
// Here it resets need_recalculation_ variable.
void
GradientLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "GradientLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when Costmap recalculation is required.
// It updates the Costmap within its window bounds.
// Inside this method the Costmap gradient is generated and is writing directly
// to the resulting Costmap master_grid without any merging with previous layers.
void
GradientLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  // master_array - is a direct pointer to the resulting master_grid.
  // master_grid - is a resulting Costmap combined from all layers.
  // By using this pointer all layers will be overwritten!
  // To work with costmap layer and merge it with other costmap layers,
  // please use costmap_ pointer instead (this is pointer to current
  // costmap layer grid) and then call one of updates methods:
  // - updateWithAddition()
  // - updateWithMax()
  // - updateWithOverwrite()
  // - updateWithTrueOverwrite()
  // In this case, using master_array pointer is equal to modifying local costmap_
  // pointer and then calling updateWithTrueOverwrite():
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  //
  // Fixing window coordinates with map size if necessary.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  // Simply computing one-by-one cost per each cell
  int gradient_index;
  for (int j = min_j; j < max_j; j++) {
    // Reset gradient_index each time when reaching the end of re-calculated window
    // by OY axis.
    gradient_index = 0;
    for (int i = min_i; i < max_i; i++) {
      int index = master_grid.getIndex(i, j);
      // setting the gradient cost
      unsigned char cost = (LETHAL_OBSTACLE - gradient_index*GRADIENT_FACTOR)%255;
      if (gradient_index <= GRADIENT_SIZE) {
        gradient_index++;
      } else {
        gradient_index = 0;
      }
      master_array[index] = cost;
    }
  }
}

}  // namespace custom_nav2_costmap_plugin

// This is the macro allowing a custom_nav2_costmap_plugin::GradientLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(custom_nav2_costmap_plugin::GradientLayer, nav2_costmap_2d::Layer)
```

Vamos comentar as diferentes partes deste código para que você entenda o que está acontecendo aqui:

**1. onInitialize()**

```cpp
void
GradientLayer::onInitialize()
{
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  need_recalculation_ = false;
  current_ = true;
}
```
Aqui você está buscando um parâmetro chamado `ativado`. Este parâmetro está no arquivo `.yaml` onde você irá carregar o plugin e seus parâmetros, neste caso habilitados da seguinte forma:

```yml
custom_gradient_layer:
    plugin: "custom_nav2_costmap_plugin/GradientLayer"
    enabled: True
```

**2. updateBounds()**

```cpp
void
GradientLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;

    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}
```
Aqui você pode ver:

* Os parâmetros de entrada para os métodos, alguns são comentados no arquivo `.cpp`:
```bash
double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/
```
* Isso evita que compiladores extra-pedantes reclamem sobre variáveis não utilizadas definidas como parâmetros. Você não está usando esses parâmetros, mas deve tê-los porque os métodos virtuais da classe base são assim. Então você faz isso para evitar avisos de parâmetros `NÃO USADOS ou até mesmo erros`.
* Este código obtém os maiores limites e é atualizado com os valores fornecidos como parâmetros.

**3. onFootprintChanged()**

```cpp
void
GradientLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "GradientLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}
```
É uma função fictícia que imprime uma mensagem quando a pegada é alterada e define o need_recalculation_ como true, para forçar novamente essa seção no método `updateBounds()`.

**4. updateCosts()**
```cpp
void
GradientLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  // master_array - is a direct pointer to the resulting master_grid.
  // master_grid - is a resulting costmap combined from all layers.
  // By using this pointer all layers will be overwritten!
  // To work with costmap layer and merge it with other costmap layers,
  // please use costmap_ pointer instead (this is pointer to current
  // costmap layer grid) and then call one of updates methods:
  // - updateWithAddition()
  // - updateWithMax()
  // - updateWithOverwrite()
  // - updateWithTrueOverwrite()
  // In this case, using master_array pointer is equal to modifying local costmap_
  // pointer and then calling updateWithTrueOverwrite():
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  //
  // Fixing window coordinates with map size if necessary.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  // Simply computing one-by-one cost per each cell
  int gradient_index;
  for (int j = min_j; j < max_j; j++) {
    // Reset gradient_index each time when reaching the end of re-calculated window
    // by OY axis.
    gradient_index = 0;
    for (int i = min_i; i < max_i; i++) {
      int index = master_grid.getIndex(i, j);
      // setting the gradient cost
      unsigned char cost = (LETHAL_OBSTACLE - gradient_index*GRADIENT_FACTOR)%255;
      if (gradient_index <= GRADIENT_SIZE) {
        gradient_index++;
      } else {
        gradient_index = 0;
      }
      master_array[index] = cost;
    }
  }
}
```
Nesse método, você está sobrescrevendo quaisquer custos que estavam anteriormente em `master_grid`.

Se você quisesse trabalhar com base no trabalho de filtros anteriores no Costmap, conforme afirma o código, você teria que usar os métodos e `costmap_` em vez do `master_grid`:

```bash
updateWithAddition()
updateWithMax()
updateWithOverwrite()
updateWithTrueOverwrite()
```

Você atualiza os custos com base na posição no Costmap e em um padrão fixo e repetitivo. Não é útil, mas é uma forma de saber como gerenciar Costmaps.

O conceito mais importante aqui é o valor de custo. Isso vai de 0 a 255, sendo 0 SEM custo e 255, o custo máximo. Ou seja: 0 SEM perigo de bater em obstáculo, `255 RISCO MUITO ALTO` de bater em obstáculo.

i e j são equivalentes a x e y no mapa. A única diferença é que não é a distância, mas os quadrados da grade, em que o mapa é dividido para calcular os Costmaps.

**Exportar MACRO do plug-in**

```cpp
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(custom_nav2_costmap_plugin::GradientLayer, nav2_costmap_2d::Layer)
```

Use isso como um plug-in na pilha ROS2 Nav2 é vital. Então aqui você define o seguinte:
```cpp
PLUGINLIB_EXPORT_CLASS(NAMESPACE_OF_OUR_CUSTOM_PLUGIN::NAME_OF_CUSTOM_CLASS, BASE_CLASS_NAMESPACE::BASE_CLASS)
```

### Etapa 3: Crie o arquivo de informações do plug-in.xml

Para que o sistema carregue seu plugin, crie um arquivo de informações no formato XML. Este arquivo terá todas as informações necessárias para encontrar a biblioteca compilada do seu plugin, o nome que você dá a ela, o namespace e a classe que ela usa.

Agora crie este arquivo e dê uma olhada:
Você pode dar o nome que quiser:

```bash
cd ~/ros2_ws/src/custom_nav2_costmap_plugin
touch gradient_layer.xml
```
> gradient_layer.xml
```xml
<library path="custom_nav2_costmap_plugin_core">
  <class name="custom_nav2_costmap_plugin/GradientLayer" type="custom_nav2_costmap_plugin::GradientLayer" base_class_type="nav2_costmap_2d::Layer">
    <description>This is an example plugin which puts repeating costs gradients to costmap</description>
  </class>
</library>
```
Vamos comentar cada TAG XML:

* **library path**: Este é o nome que você dá à biblioteca ao compilá-la. Você define isso dentro do `CMakelists`.txt, para ser revisado na próxima etapa.
* class type: indica o `CUSTOM_PLUGIN_NAMESPACE` e o `CUSTOM_CLASS` que você definiu nos arquivos `.cpp` e `.hpp`. No seu caso, é `CUSTOM_PLUGIN_NAMESPACE=custom_nav2_costmap_plugin`, `CUSTOM_CLASS=GradientLayer`.
* class base_class_type: Aqui você declara novamente a BASE CLASS com seu namespace no qual sua classe personalizada é baseada.
* class name: Esta tag é opcional, se não for colocada, o nome dado será o mesmo do tipo de classe. Mas é melhor usá-lo porque torna seu código mais legível. Nesse caso, o nome é `custom_nav2_costmap_plugin/GradientLayer`.

### Etapa 4: configurar o `CMakelists.txt` e o `package.xml` para compilação
Este último passo é necessário para dizer ao ROS para compilar seu plugin como uma biblioteca e exportá-lo para o sistema de plugins, para que possa ser encontrado e usado.

> CMakelists.txt
```cmake
cmake_minimum_required(VERSION 3.8)
project(custom_nav2_costmap_plugin)

set(lib_name ${PROJECT_NAME}_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

# You set a list of dependencies to make it more compact
set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pluginlib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# === Build ===

add_library(${lib_name} SHARED
            src/gradient_layer.cpp)


include_directories(include)

# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)
      

# === Ament work ===

# pluginlib_export_plugin_description_file() installs gradient_layer.xml
# file into "share" directory and sets ament indexes for it.
# This allows the plugin to be discovered as a plugin of required type.
pluginlib_export_plugin_description_file(nav2_costmap_2d gradient_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})


ament_package()
```
**1. Definir biblioteca de nomes**

Primeiro, defina uma variável chamada lib_name que contém o nome que você selecionou para sua biblioteca de plug-ins quando compilada:
```cpp
set(lib_name ${PROJECT_NAME}_core)
```
Neste caso, o nome é `${PROJECT_NAME}_core = custom_nav2_costmap_plugin_core`

É por isso que no arquivo de informações do plug-in `gradient_layer.xml`, você define `custom_nav2_costmap_plugin_core` como o caminho.

```xml
<library path="custom_nav2_costmap_plugin_core">
```
**2. definir lista de dependências de nomes**

```cpp
set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pluginlib)
```
Nomeie `dep_pkgs` a lista de dependências em sua biblioteca que você usará depois.

**3. adicione a biblioteca**

```cpp
add_library(${lib_name} SHARED
            src/gradient_layer.cpp)
```
Declare aqui Compile o arquivo `gradient_layer.cpp` em uma biblioteca chamada `lib_name=custom_nav2_costmap_plugin_core.`

**4. incluir definição de pasta**

Inclua os arquivos de cabeçalho dentro da pasta de inclusão do pacote
```cpp
include_directories(include)
```
instale a biblioteca compilada
```cpp
install(TARGETS ${lib_name}
        DESTINATION lib)
```
**5. disponibilizar a biblioteca para o sistema de plugins**
```cpp
pluginlib_export_plugin_description_file(nav2_costmap_2d gradient_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
```

> package.xml

Diga ao pacote onde encontrar todas as informações relacionadas ao seu plugin:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>custom_nav2_costmap_plugin</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="duckfrost@gmail.com">tgrip</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>nav2_costmap_2d</depend>
  <depend>pluginlib</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <costmap_2d plugin="${prefix}/gradient_layer.xml" />
    <build_type>ament_cmake</build_type>
  </export>

</package>
```
Esta é a única linha alterada:
```xml
<costmap_2d plugin="${prefix}/gradient_layer.xml" />
```
### Passo 5: configurar, compilar e testar

Crie um novo lançamento do pathplanner que carregue seu plug-in personalizado. Para fazer isso, crie um novo lançamento e alguns novos arquivos de configuração:

```bash
cd ~/ros2_ws/
touch ~/ros2_ws/src/path_planner_server/launch/pathplanner_custom_costmap_plugin.launch.py
mkdir ~/ros2_ws/src/path_planner_server/config/custom_costmap
cd ~/ros2_ws/src/path_planner_server/config/custom_costmap
touch controller.yaml
touch planner_server.yaml
```
Você adicionará o plug-in no mapa de custo local e no mapa de custo global.
> controller.yaml
```yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    # controller_frequency: 20.0
    controller_frequency: 5.0
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
      critics: ["RotateToGoal", "Oscillation", "ObstacleFootprint", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
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

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_footprint
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.4
      # plugins: ["voxel_layer", "inflation_layer"]
      plugins: ["voxel_layer", "custom_gradient_layer"]
      # inflation_layer:
      #   plugin: "nav2_costmap_2d::InflationLayer"
      #   cost_scaling_factor: 3.0
      #   inflation_radius: 0.55
      custom_gradient_layer:
        plugin: "custom_nav2_costmap_plugin/GradientLayer"
        enabled: True
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        map_subscribe_transient_local: True
      always_send_full_costmap: True
```
Como você pode ver, você substituiu a camada de inflação pela camada `custom_gradient`.
Você usa o nome `custom_nav2_costmap` `plugin/Gradient` Layer especificado no arquivo `gradient_layer.xml`.
```yaml
# plugins: ["voxel_layer", "inflation_layer"]
plugins: ["voxel_layer", "custom_gradient_layer"]

...
# inflation_layer:
#   plugin: "nav2_costmap_2d::InflationLayer"
#   cost_scaling_factor: 3.0
#   inflation_radius: 0.55
...
custom_gradient_layer:
    plugin: "custom_nav2_costmap_plugin/GradientLayer"
    enabled: True
```
> planner_server.yaml
```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_footprint
      use_sim_time: True
      robot_radius: 0.3
      resolution: 0.05
      track_unknown_space: true
      # plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      plugins: ["static_layer", "obstacle_layer", "custom_gradient_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      always_send_full_costmap: True
      obstacle_layer:
          plugin: "nav2_costmap_2d::ObstacleLayer"
          enabled: True
          observation_sources: scan
          scan:
            topic: /scan
            max_obstacle_height: 2.0
            clearing: True
            marking: True
            data_type: "LaserScan"
            raytrace_max_range: 3.0
            raytrace_min_range: 0.0
            obstacle_max_range: 2.5
            obstacle_min_range: 0.0
      # inflation_layer:
      #   plugin: "nav2_costmap_2d::InflationLayer"
      #   cost_scaling_factor: 3.0
      #   inflation_radius: 0.55
      custom_gradient_layer:
        plugin: "custom_nav2_costmap_plugin/GradientLayer"
        enabled: True
```
E aqui, no mesmo procedimento, você substituiu a inflation_layer por sua custom_gradient_layer. Você não precisaria adicionar nenhum dos outros plug-ins porque está sobrescrevendo seus dados, devido à maneira como definiu seu método updateCosts().

> pathplanner_custom_costmap_plugin.launch.py
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Custom Plugin
    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'custom_costmap', 'controller.yaml')
    # Custom Plugin
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'custom_costmap', 'planner_server.yaml')
    
    # Standard
    default_bt_xml_path = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'behavior.xml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator.yaml')
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
            parameters=[bt_navigator_yaml, {'default_bt_xml_filename': default_bt_xml_path}]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['controller_server',
                                        'planner_server',
                                        'recoveries_server',
                                        'bt_navigator']}])
    ])
```
Altere os arquivos dos quais você obtém e carregue os parâmetros para os nós **controller_server** e **planner_server**.
```python
# Custom Plugin
controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'custom_costmap', 'controller.yaml')
# Custom Plugin
planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'custom_costmap', 'planner_server.yaml')
```
> setup.py
Para encontrar a nova pasta **config/custom_costmap** e seus arquivos, adicione-os ao `setup.py` da seguinte forma:
```python
from setuptools import setup
import os
from glob import glob


package_name = 'path_planner_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config/custom_costmap'), glob('config/custom_costmap/*.yaml')),
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
Adicionando esta linha de código:
```python
(os.path.join('share', package_name, 'config/custom_costmap'), glob('config/custom_costmap/*.yaml')),
```
Esta linha adiciona o config/custom_costmap e os arquivos dentro dele.
#### Compilar
É hora de compilar seu plugin e ver o que você obtém ao usá-lo. Você também precisa compilar o pacote path_planner_server para que suas alterações sejam instaladas e acessíveis.

Primeiro, compile-o:
```bash
cd ~/ros2_ws/
colcon build --packages-select custom_nav2_costmap_plugin path_planner_server
source install/setup.bash
```
Nenhum erro deve aparecer. Caso contrário, verifique o código e verifique se não perdeu nada.

#### Iniciar o servidor de localização

```bash
cd ~/ros2_ws
source install/setup.bash;reset;ros2 launch localization_server localization.launch.py
```
#### Iniciar planejamento do caminho
```bash
cd ~/ros2_ws
source install/setup.bash;reset;ros2 launch path_planner_server pathplanner_custom_costmap_plugin.launch.py
```

Agora você deve ir para as Ferramentas Gráficas (deve abrir automaticamente) e alterar o arquivo de configuração do RVIZ para o planejamento do caminho. Deve estar no caminho `~/ros2_ws/src/path_planner_server/rviz_config/pathplanning.rviz`.

Vá para `File` -> `Open Config` e selecione esse arquivo.

Agora você deve ver algo assim:

Como você pode ver, o controlador não pode mover o robô. Isso porque o Costmap possui uma configuração que não permite que o robô se movimente livremente.

Simular uma pessoa andando por aí de forma que o robô tenha que planejar em torno dela dinamicamente.

* Criar um novo plugin Costmap
* Altere o método `updateCosts()` para gerar uma pequena área de caixa que se move, simulando uma pessoa ou objeto se movendo pela cena.
* Adicione este plug-in do Costmap apenas à configuração global do Costmap.

Defina os nomes dos arquivos e outras tags, para que você possa seguir melhor a solução se ficar preso:

1. my_custom_costmap.cpp
2. my_custom_costmap.hpp
3. Nome da classe personalizada = MyCustomCostmap
4. nome do namespace da classe personalizada = my_custom_costmap_namespace_name
5. nome da biblioteca em CMkelists.txt = my_custom_costmap_core
6. Nome do plug-in no arquivo info.xml = MyCrazyCustomCostmapLayer_Plugin_Name
7. Nome do arquivo info.xml = my_custom_costmap_plugininfo.xml
8. O nome da pasta de configuração do planejador = my_custom_crazy_costmap
9. O nome de lançamento do planejador = pathplanner_custom_crazy_costmap.launch.py

Para adicioná-lo ao package.xml, adicione esta linha:
```xml
<costmap_2d plugin="${prefix}/my_custom_costmap_plugininfo.xml" />
```
Uma maneira sugerida de selecionar quais quadrados da grade do Costmap estão dentro do obstáculo em movimento pode ser o seguinte:
```cpp
bool
MyCustomCostmap::pointInsideCircle(int x_point, int y_point)
{

  int delta_x = abs(circle_p_x - x_point);
  int delta_y = abs(circle_p_y - y_point);
  bool delta_x_in = (delta_x <= obstacle_radius);
  bool delta_y_in = (delta_y <= obstacle_radius);
  bool inside_circle = (delta_x_in && delta_y_in);

  // RCLCPP_WARN(rclcpp::get_logger(
  //     "nav2_costmap_2d"), "Point Inside Cicle = %d", inside_circle);

  return inside_circle;
}
```
E como você atribui os valores pode ser feito desta forma:
```cpp
bool result = pointInsideCircle(i, j);

if (result){
    master_array[index] = 200;
}else{
    master_array[index] = 1;
}
```

> pathplanner_custom_crazy_costmap.launch.py

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Custom Plugin
    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'my_custom_crazy_costmap', 'controller.yaml')
    # Custom Plugin
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'my_custom_crazy_costmap', 'planner_server.yaml')
    
    # Standard
    default_bt_xml_path = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'behavior.xml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator.yaml')
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
            parameters=[bt_navigator_yaml, {'default_bt_xml_filename': default_bt_xml_path}]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['controller_server',
                                        'planner_server',
                                        'recoveries_server',
                                        'bt_navigator']}])
    ])
```
> setup.py
```python
from setuptools import setup
import os
from glob import glob


package_name = 'path_planner_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config/custom_costmap'), glob('config/custom_costmap/*.yaml')),
        (os.path.join('share', package_name, 'config/my_custom_crazy_costmap'), glob('config/my_custom_crazy_costmap/*.yaml')),
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
> my_custom_crazy_costmap/controller.yaml
```yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    # controller_frequency: 20.0
    controller_frequency: 5.0
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
      critics: ["RotateToGoal", "Oscillation", "ObstacleFootprint", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
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

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_footprint
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.4
      plugins: ["voxel_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        map_subscribe_transient_local: True
      always_send_full_costmap: True
```
> my_custom_crazy_costmap/planner_server.yaml¶
```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_footprint
      use_sim_time: True
      robot_radius: 0.3
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "my_crazy_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      always_send_full_costmap: True
      obstacle_layer:
          plugin: "nav2_costmap_2d::ObstacleLayer"
          enabled: True
          observation_sources: scan
          scan:
            topic: /scan
            max_obstacle_height: 2.0
            clearing: True
            marking: True
            data_type: "LaserScan"
            raytrace_max_range: 3.0
            raytrace_min_range: 0.0
            obstacle_max_range: 2.5
            obstacle_min_range: 0.0
      my_crazy_layer:
        plugin: "MyCrazyCustomCostmapLayer_Plugin_Name"
        enabled: True
        x_oscilation_height : 700.0
        y_oscilation_range : 100.0 
        obstacle_radius : 15
```
> my_custom_costmap.cpp
```cpp
#include "custom_nav2_costmap_plugin/my_custom_costmap.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace my_custom_costmap_namespace_name
{

MyCustomCostmap::MyCustomCostmap()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
MyCustomCostmap::onInitialize()
{
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  declareParameter("x_oscilation_height", rclcpp::ParameterValue(10.0));
  node->get_parameter(name_ + "." + "x_oscilation_height", x_oscilation_height);

  declareParameter("y_oscilation_range", rclcpp::ParameterValue(10.0));
  node->get_parameter(name_ + "." + "y_oscilation_range", y_oscilation_range);

  declareParameter("obstacle_radius", rclcpp::ParameterValue(30));
  node->get_parameter(name_ + "." + "obstacle_radius", obstacle_radius);

  need_recalculation_ = false;
  current_ = true;
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
MyCustomCostmap::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{

  last_min_x_ = *min_x;
  last_min_y_ = *min_y;
  last_max_x_ = *max_x;
  last_max_y_ = *max_y;
  *min_x = -std::numeric_limits<float>::max();
  *min_y = -std::numeric_limits<float>::max();
  *max_x = std::numeric_limits<float>::max();
  *max_y = std::numeric_limits<float>::max();

  need_recalculation_ = true;

}

// The method is called when footprint was changed.
// Here it resets need_recalculation_ variable.
void
MyCustomCostmap::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "MyCustomCostmap::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void
MyCustomCostmap::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  // master_array - is a direct pointer to the resulting master_grid.
  // master_grid - is a resulting costmap combined from all layers.
  // By using this pointer all layers will be overwritten!
  // To work with costmap layer and merge it with other costmap layers,
  // please use costmap_ pointer instead (this is pointer to current
  // costmap layer grid) and then call one of updates methods:
  // - updateWithAddition()
  // - updateWithMax()
  // - updateWithOverwrite()
  // - updateWithTrueOverwrite()
  // In this case, using master_array pointer is equal to modifying local costmap_
  // pointer and then calling updateWithTrueOverwrite():
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  //
  // Fixing window coordinates with map size if necessary.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);



  // You calculate the new position of circle
  circle_p_x = x_oscilation_height;

  // Calculate center
  circle_p_y = (max_j - min_j)/2;

  if (delta_flag){
    circle_p_y_delta += 10;
  }else{
    circle_p_y_delta -= 10;
  }

  circle_p_y += circle_p_y_delta;
    
  if (circle_p_y >= (((max_j - min_j)/2) + y_oscilation_range)){
    delta_flag = false;
  }else if (circle_p_y <= (((max_j - min_j)/2) - y_oscilation_range))
  {
    delta_flag = true;
  }

  RCLCPP_WARN(rclcpp::get_logger(
  "nav2_costmap_2d"), "circle_p_x = %f", circle_p_x);
  RCLCPP_WARN(rclcpp::get_logger(
  "nav2_costmap_2d"), "circle_p_y = %f", circle_p_y);


  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = master_grid.getIndex(i, j);

      bool result = pointInsideCircle(i, j);

      if (result){
        master_array[index] = 200;
      }else{
        master_array[index] = 1;
      }
      
    }
  }



  
}

bool
MyCustomCostmap::pointInsideCircle(int x_point, int y_point)
{

  int delta_x = abs(circle_p_x - x_point);
  int delta_y = abs(circle_p_y - y_point);
  bool delta_x_in = (delta_x <= obstacle_radius);
  bool delta_y_in = (delta_y <= obstacle_radius);
  bool inside_circle = (delta_x_in && delta_y_in);

  // RCLCPP_WARN(rclcpp::get_logger(
  //     "nav2_costmap_2d"), "Point Inside Cicle = %d", inside_circle);

  return inside_circle;
}

}  // namespace my_custom_costmap_namespace_name

// This is the macro allowing a my_custom_costmap_namespace_name::MyCustomCostmap class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_custom_costmap_namespace_name::MyCustomCostmap, nav2_costmap_2d::Layer)
```
> my_custom_costmap.hpp
```cpp
#ifndef MY_CUSTOM_COSTMAP_HPP_
#define MY_CUSTOM_COSTMAP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include <math.h>

namespace my_custom_costmap_namespace_name
{

class MyCustomCostmap : public nav2_costmap_2d::Layer
{
public:
  MyCustomCostmap();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  virtual void onFootprintChanged();

  virtual bool isClearable() {return false;}

  bool pointInsideCircle(int x_point, int y_point);

private:
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  // Indicates that the entire gradient should be recalculated next time.
  bool need_recalculation_;

  // Size of gradient in cells
  int GRADIENT_SIZE = 20;
  // Step of increasing cost per one cell in gradient
  int GRADIENT_FACTOR = 10;
  
  // Init Crazy Circle pose
  float circle_p_x;
  float circle_p_y;
  float circle_p_y_delta = 0;
  bool delta_flag = true;
  int obstacle_radius = 30;
  float x_oscilation_height;
  float y_oscilation_range;
};

}  // namespace my_custom_costmap_namespace_name

#endif  // MY_CUSTOM_COSTMAP_HPP_
```
> CMakelists.txt
```cmake
cmake_minimum_required(VERSION 3.8)
project(custom_nav2_costmap_plugin)

set(lib_name ${PROJECT_NAME}_core)
set(my_custom_costmap_lib_name my_custom_costmap_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

# You set a list of dependencies to make it more compact
set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pluginlib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# === Build ===

add_library(${lib_name} SHARED
            src/gradient_layer.cpp)

add_library(${my_custom_costmap_lib_name} SHARED
            src/my_custom_costmap.cpp)

include_directories(include)

# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)
      
install(TARGETS ${my_custom_costmap_lib_name}
        DESTINATION lib)

# === Ament work ===

# pluginlib_export_plugin_description_file() installs gradient_layer.xml
# file into "share" directory and sets ament indexes for it.
# This allows the plugin to be discovered as a plugin of required type.
pluginlib_export_plugin_description_file(nav2_costmap_2d gradient_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})

pluginlib_export_plugin_description_file(nav2_costmap_2d my_custom_costmap_plugininfo.xml)
ament_target_dependencies(${my_custom_costmap_lib_name} ${dep_pkgs})

ament_package()
```
> package.xml
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>custom_nav2_costmap_plugin</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="duckfrost@gmail.com">tgrip</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>nav2_costmap_2d</depend>
  <depend>pluginlib</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <costmap_2d plugin="${prefix}/gradient_layer.xml" />
    <costmap_2d plugin="${prefix}/my_custom_costmap_plugininfo.xml" />
    <build_type>ament_cmake</build_type>
  </export>

</package>
```
> my_custom_costmap_plugininfo.xml
```xml
<library path="my_custom_costmap_core">
  <class name="MyCrazyCustomCostmapLayer_Plugin_Name" type="my_custom_costmap_namespace_name::MyCustomCostmap" base_class_type="nav2_costmap_2d::Layer">
    <description>This is a Crazy costmap layer created for fun</description>
  </class>
</library>
```
Repositorio completo [aqui](https://bitbucket.org/theconstructcore/nav2_plugins_solutions/src/master/).
## Plug-in do planejador
Agora você verá um novo exemplo de Plugin do Planner que cria apenas Caminhos Retos, sem considerar obstáculos. Então, dê uma olhada nas etapas. Este exemplo será menos explícito porque a maioria dos conceitos são os mesmos em todos os plugins:

### Passo 1: Criar o pacote
Crie um pacote onde você armazenará todos os plugins para planejadores. Execute os seguintes comandos:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake custom_nav2_planner_plugin --dependencies rclcpp rclcpp_action rclcpp_lifecycle std_msgs visualization_msgs nav2_util nav2_msgs nav_msgs geometry_msgs builtin_interfaces tf2_ros nav2_costmap_2d nav2_core pluginlib
cd ~/ros2_ws
colcon build --packages-select custom_nav2_planner_plugin
source install/setup.bash
```
### Etapa 2: Crie o arquivo de origem e o arquivo de cabeçalho
Agora, crie os arquivos `.cpp` e `.hpp` que definem o plugin:
```bash
cd ~/ros2_ws/src/custom_nav2_planner_plugin
touch src/straight_line_planner.cpp
touch include/custom_nav2_planner_plugin/straight_line_planner.hpp
```
> straight_line_planner.hpp
```cpp
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions, and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions, and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING BUT NOT
 *  LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#ifndef NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
#define NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_straightline_planner_namespace_name
{

class StraightLine : public nav2_core::GlobalPlanner
{
public:
  StraightLine() = default;
  ~StraightLine() = default;

  // plugin configure
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // plugin cleanup
  void cleanup() override;

  // plugin activate
  void activate() override;

  // plugin deactivate
  void deactivate() override;

  // This method creates path for given start and goal pose.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // node ptr
  nav2_util::LifecycleNode::SharedPtr node_;

  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;

  // The global frame of the costmap
  std::string global_frame_, name_;

  double interpolation_resolution_;
};

}  // namespace nav2_straightline_planner_namespace_name

#endif  // NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
```
Todos os métodos que você substituirá da classe base `nav2_core::GlobalPlanner` estão dentro da seção pública de sua classe:
```cpp
public:
  StraightLine() = default;
  ~StraightLine() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;


  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;
```

* `configure()`: **OBRIGATÓRIO**. Este método é chamado quando o nó está no estado de configuração. Isso significa que aqui é onde você deve carregar todos os parâmetros e inicializar todas as variáveis.
* `activate()`: **OBRIGATÓRIO**. Quando você ativa isso, isso é executado, então coloque tudo o que você precisa aqui antes que o plugin seja ativado.
* `deactivate()`: **OBRIGATÓRIO**.

* `cleanup()`: **OBRIGATÓRIO**. Quando você for para o estado limpo, libere tarefas de memória e similares.

* `criarPlano()`: **OBRIGATÓRIO**. Os métodos são chamados quando o servidor deseja uma geração de caminho global com base em start e goal_pose. Este método retorna um `nav_msgs::msg::Path` que será usado como o caminho global planejado.
Agora, dê uma olhada no código para o código `.cpp`:
> straight_line_planner.cpp
```cpp
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions, and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions, and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING BUT NOT
 *  LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "custom_nav2_planner_plugin/straight_line_planner.hpp"

namespace nav2_straightline_planner_namespace_name
{

void StraightLine::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void StraightLine::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLine::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLine::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

nav_msgs::msg::Path StraightLine::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  // calculating the number of loops for current value of interpolation_resolution_
  int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y) /
    interpolation_resolution_;
  double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  for (int i = 0; i < total_number_of_loop; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = start.pose.position.x + x_increment * i;
    pose.pose.position.y = start.pose.position.y + y_increment * i;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }

  global_path.poses.push_back(goal);

        RCLCPP_WARN(
      node_->get_logger(), "Plann Straight line DONE");

  return global_path;
}

}  // namespace nav2_straightline_planner_namespace_name

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner_namespace_name::StraightLine, nav2_core::GlobalPlanner)
```
Vamos comentar as diferentes partes deste código para entender o que está acontecendo aqui. Existem principalmente dois métodos que valem a pena comentar:

1. **configure()**
```cpp
void StraightLine::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}
```
Aqui você está obtendo o valor do parâmetro `interpolation_resolution` que será usado para a geração do caminho em linha reta. Ele será definido no arquivo YAML:
```yaml
planner_plugins: ["GridBased"]    
GridBased:
  plugin: StraightLineCustomPlugin
  interpolation_resolution: 0.1
```
2. **createPlan()**
```cpp
nav_msgs::msg::Path StraightLine::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  // calculating the number of loops for current value of interpolation_resolution_
  int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y) /
    interpolation_resolution_;
  double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  for (int i = 0; i < total_number_of_loop; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = start.pose.position.x + x_increment * i;
    pose.pose.position.y = start.pose.position.y + y_increment * i;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }

  global_path.poses.push_back(goal);

        RCLCPP_WARN(
      node_->get_logger(), "Plann Straight line DONE");

  return global_path;
}
```
Aqui você pode ver:

Os parâmetros de entrada para o método são start e goal
Você está calculando a distância entre esses dois pontos 2D e dividindo-a pela interpolação_resolução:
```cpp
int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y) /
    interpolation_resolution_;
```
Isso nos dará o número de partes em que o caminho em linha reta será dividido. Quanto maior a distância, maior o número de peças que ele terá no caminho.
```cpp
for (int i = 0; i < total_number_of_loop; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = start.pose.position.x + x_increment * i;
    pose.pose.position.y = start.pose.position.y + y_increment * i;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }
```
E aqui, você está gerando os pontos desse caminho, usando incrementos linearmente para gerar uma linha de caminho perfeitamente reta.\

### Etapa 3: Crie o arquivo de informações do plug-in.xml
```bash
cd ~/ros2_ws/src/custom_nav2_planner_plugin
touch straight_line_plugin_info.xml
```
> straight_line_plugin_info.xml
```xml
<library path="straight_line_planner_core">
	<class name="StraightLineCustomPlugin" type="nav2_straightline_planner_namespace_name::StraightLine" base_class_type="nav2_core::GlobalPlanner">
	  <description>This is an example plugin which produces straight line path.</description>
	</class>
</library>
```
### Passo 4: Configure o CMakelists.txt e o package.xml para compilação
> CMakelists.txt
```cmake
cmake_minimum_required(VERSION 3.8)
project(custom_nav2_planner_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
)

set(straight_plugin_name straight_line_planner_core)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  nav2_costmap_2d
  nav2_core
  pluginlib
)

# StraightLine plugin related stuff 
add_library(${straight_plugin_name} SHARED
  src/straight_line_planner.cpp
)

ament_target_dependencies(${straight_plugin_name}
  ${dependencies}
)

target_compile_definitions(${straight_plugin_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core straight_line_plugin_info.xml)

install(TARGETS ${straight_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${straight_plugin_name})
ament_export_dependencies(${dependencies})
ament_package()
```
Mesmo procedimento exceto para alguns elementos:

`target_compile_definitions`: Isso ocorre devido à necessidade de desabilitar as funções BOOST ao compilar este plug-in.
```cmake
target_compile_definitions(${straight_plugin_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
```
> package.xml
Diga ao pacote onde encontrar todas as informações relacionadas ao seu plugin:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>custom_nav2_planner_plugin</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="duckfrost@gmail.com">tgrip</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_action</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>std_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>nav2_util</depend>
  <depend>nav2_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>builtin_interfaces</depend>
  <depend>tf2_ros</depend>
  <depend>nav2_costmap_2d</depend>
  <depend>nav2_core</depend>
  <depend>pluginlib</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
    <nav2_core plugin="${prefix}/straight_line_plugin_info.xml" />
  </export>
</package>
```
### Passo 5: Configurar, compilar e testar

**configurar**
Crie um novo lançamento do pathplanner que carregue seu plug-in personalizado. Para fazer isso, crie um novo lançamento e alguns novos arquivos de configuração:
```bash
cd ~/ros2_ws/
touch ~/ros2_ws/src/path_planner_server/launch/pathplanner_straightlineplanner_plugin.launch.py
mkdir ~/ros2_ws/src/path_planner_server/config/straightline_planner
cd ~/ros2_ws/src/path_planner_server/config/straightline_planner
touch planner_server.yaml
```
Aqui você só precisa do planner_server.yaml porque seu plugin está no servidor planner que gerou os caminhos globais.
> planner_server.yaml
```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True    
    planner_plugins: ["GridBased"]    
    GridBased:
      plugin: StraightLineCustomPlugin
      interpolation_resolution: 0.1

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_footprint
      use_sim_time: True
      robot_radius: 0.3
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True
      obstacle_layer:
          plugin: "nav2_costmap_2d::ObstacleLayer"
          enabled: True
          observation_sources: scan
          scan:
            topic: /scan
            max_obstacle_height: 2.0
            clearing: True
            marking: True
            data_type: "LaserScan"
            raytrace_max_range: 3.0
            raytrace_min_range: 0.0
            obstacle_max_range: 2.5
            obstacle_min_range: 0.0
```

Como você pode ver, você não mudou o nome do parâmetro. Você usa o `GridBaased`. Por enquanto, se isso for alterado, dará erros.
Você está carregando o plug-in com o nome que deu a ele no `straight_line_plugin_info.xml`, StraightLineCustomPlugin.
```yaml
planner_plugins: ["GridBased"]    
GridBased:
  plugin: StraightLineCustomPlugin
  interpolation_resolution: 0.1
```
> pathplanner_straightlineplanner_plugin.launch.py
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Custom Plugin
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'straightline_planner', 'planner_server.yaml')
    
    # Standard
    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
    default_bt_xml_path = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'behavior.xml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator.yaml')
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
            parameters=[bt_navigator_yaml, {'default_bt_xml_filename': default_bt_xml_path}]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['controller_server',
                                        'planner_server',
                                        'recoveries_server',
                                        'bt_navigator']}])
    ])
```
Como você pode ver, você apenas altera o `planner_yaml`. O restante é a configuração padrão.
```bash
# Custom Plugin
planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'straightline_planner', 'planner_server.yaml')
```
> setup.py
Para encontrar a nova pasta **config/custom_costmap** e seus arquivos, você precisa adicioná-los ao `setup.py` da seguinte forma:
```python
from setuptools import setup
import os
from glob import glob


package_name = 'path_planner_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config/custom_costmap'), glob('config/custom_costmap/*.yaml')),
        (os.path.join('share', package_name, 'config/my_custom_crazy_costmap'), glob('config/my_custom_crazy_costmap/*.yaml')),
        (os.path.join('share', package_name, 'config/straightline_planner'), glob('config/straightline_planner/*.yaml')),
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
Adicionando esta linha de código:
```python
(os.path.join('share', package_name, 'config/straightline_planner'), glob('config/straightline_planner/*.yaml')),
```
Esta linha adiciona o config/straightline_planner e os arquivos dentro dele.

#### Compilar
É hora de compilar seu plugin e ver o que você obtém ao usá-lo.

Você também deve compilar o pacote path_planner_server para que suas alterações sejam instaladas e acessíveis.

Primeiro, compile-o:
```bash
CD ~/ros2_ws/
colcon build --packages-select custom_nav2_planner_plugin path_planner_server
fonte install/setup.bash
```
Nenhum erro deve aparecer. Caso contrário, verifique o código e verifique se não perdeu nada.

#### Iniciar localização
Executar no Shell #1
```bash
cd ~/ros2_ws
source install/setup.bash;reset;ros2 launch localization_server localization.launch.py
```
#### Iniciar o servidor de planejamento
Executar no Shell #2
```bash
cd ~/ros2_ws
source install/setup.bash;reset;ros2 launch path_planner_server pathplanner_straightlineplanner_plugin.launch.py
```
Agora você deve acessar as Ferramentas Gráficas (deve abrir automaticamente) e alterar o arquivo de configuração do RVIZ para o planejamento do caminho. Deve estar no caminho `~/ros2_ws/src/path_planner_server/rviz_config/pathplanning.rviz`.

Vá para File->Open Config e selecione esse arquivo.

Todos os caminhos são `LINHAS RETAS`.

Mesmo aqueles que passam por `OBSTÁCULOS`. Você pode ver que ele não planeja o caminho global ao seu redor. MAS o robô para porque o Planejamento do Caminho LOCAL entra em ação. Mas esse local não se recupera bem porque, no final das contas, é o Caminho Global que faz tudo funcionar e se adaptar.

Defina os nomes dos arquivos e outras tags para que você possa seguir melhor a solução se ficar preso:

* wavy_line_planner.cpp
* wavy_line_planner.hpp
* Nome da classe personalizada = WavyLine
* nome do namespace da classe personalizada = nav2_wavyline_planner_namespace_name
* nome da biblioteca em CMakelists.txt = wavy_line_planner_core
* Nome do plugin no arquivo info.xml = WavyLineCustomPlugin
* Nome do arquivo info.xml = wavy_line_plugin_info.xml
* O nome da pasta de configuração do planejador = wavyline_planner
* O nome de lançamento do planejador = pathplanner_wavyline_planner_plugin.launch.py
* Uma sugestão é usar `SIN` e `COS` para gerar algum tipo de trajetória ondulada oscilante:

```cpp
#include <math.h>

#define PI 3.14159265

...


int A = 4;
float frequency = 2.0;
x_sin_increment = A * x_increment * sin (i_angle*frequency);
y_cos_increment = A * y_increment * cos (i_angle*frequency);


...


pose.pose.position.x = start.pose.position.x + x_increment * i + x_sin_increment;
pose.pose.position.y = start.pose.position.y + y_increment * i + y_cos_increment;
```
Mas, novamente, você é livre para fazer qualquer caminho, a única restrição é que NÃO PODE ser RETO.

> wavy_line_planner.cpp
```cpp
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions, and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions, and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING BUT NOT
 *  LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "custom_nav2_planner_plugin/wavy_line_planner.hpp"

namespace nav2_wavyline_planner_namespace_name
{

void WavyLine::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void WavyLine::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void WavyLine::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void WavyLine::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

nav_msgs::msg::Path WavyLine::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  // calculating the number of loops for current value of interpolation_resolution_
  int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y) /
    interpolation_resolution_;
  double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  for (int i = 0; i < total_number_of_loop; ++i) {
    geometry_msgs::msg::PoseStamped pose;

    // You only create zig zag if total_number_of_loop > 3
    float x_sin_increment = 0.0;
    float y_cos_increment = 0.0;
    float angle_increment = 2.0 * PI / total_number_of_loop;
    float i_angle = 0.0;

    if (total_number_of_loop > 3){
      

      i_angle = angle_increment * i;
      
      int A = 4;
      float frequency = 2.0;
      x_sin_increment = A * x_increment * sin (i_angle*frequency);
      y_cos_increment = A * y_increment * cos (i_angle*frequency);

      RCLCPP_WARN(
      node_->get_logger(), ">>>>>>>> SIN X=%f,COS Y=%f, ANGLE=%f, index=%i", x_sin_increment, y_cos_increment, i_angle, i);
    }else{

    }


    pose.pose.position.x = start.pose.position.x + x_increment * i + x_sin_increment;
    pose.pose.position.y = start.pose.position.y + y_increment * i + y_cos_increment;

    RCLCPP_WARN(
      node_->get_logger(), "Pose X=%f, Y=%f, index=%i", pose.pose.position.x, pose.pose.position.y, i);

    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }

  global_path.poses.push_back(goal);

        RCLCPP_WARN(
      node_->get_logger(), ">>>>>>>>>>> Plann WAVY -/-/-/- line DONE");

  return global_path;
}

}  // namespace nav2_wavyline_planner_namespace_name

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_wavyline_planner_namespace_name::WavyLine, nav2_core::GlobalPlanner)
```

> wavy_line_planner.hpp

```cpp
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions, and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions, and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING BUT NOT
 *  LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#ifndef NAV2_WAVYLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
#define NAV2_WAVYLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include <math.h>

#define PI 3.14159265

namespace nav2_wavyline_planner_namespace_name
{

class WavyLine : public nav2_core::GlobalPlanner
{
public:
  WavyLine() = default;
  ~WavyLine() = default;

  // plugin configure
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // plugin cleanup
  void cleanup() override;

  // plugin activate
  void activate() override;

  // plugin deactivate
  void deactivate() override;

  // This method creates path for given start and goal pose.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // node ptr
  nav2_util::LifecycleNode::SharedPtr node_;

  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;

  // The global frame of the costmap
  std::string global_frame_, name_;

  double interpolation_resolution_;
};

}  // namespace nav2_wavyline_planner_namespace_name

#endif  // NAV2_WAVYLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
```

> wavy_line_plugin_info.xml

```xml
<library path="wavy_line_planner_core">
	<class name="WavyLineCustomPlugin" type="nav2_wavyline_planner_namespace_name::WavyLine" base_class_type="nav2_core::GlobalPlanner">
	  <description>This is an example plugin which produces Wavy line path.</description>
	</class>
</library>
```

> CMakelists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(custom_nav2_planner_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
)

set(straight_plugin_name straight_line_planner_core)
set(wavy_plugin_name wavy_line_planner_core)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  nav2_costmap_2d
  nav2_core
  pluginlib
)

# StraightLine plugin related stuff 
add_library(${straight_plugin_name} SHARED
  src/straight_line_planner.cpp
)

ament_target_dependencies(${straight_plugin_name}
  ${dependencies}
)

target_compile_definitions(${straight_plugin_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core straight_line_plugin_info.xml)

install(TARGETS ${straight_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# WavyLine plugin related stuff 
add_library(${wavy_plugin_name} SHARED
  src/wavy_line_planner.cpp
)

ament_target_dependencies(${wavy_plugin_name}
  ${dependencies}
)

target_compile_definitions(${wavy_plugin_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core wavy_line_plugin_info.xml)

install(TARGETS ${wavy_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# install(FILES straight_line_plugin_info.xml wavy_line_plugin_info.xml
#   DESTINATION share/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${straight_plugin_name})
ament_export_libraries(${wavy_plugin_name})
ament_export_dependencies(${dependencies})
ament_package()
```

> package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>custom_nav2_planner_plugin</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="duckfrost@gmail.com">tgrip</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_action</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>std_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>nav2_util</depend>
  <depend>nav2_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>builtin_interfaces</depend>
  <depend>tf2_ros</depend>
  <depend>nav2_costmap_2d</depend>
  <depend>nav2_core</depend>
  <depend>pluginlib</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
    <nav2_core plugin="${prefix}/straight_line_plugin_info.xml" />
    <nav2_core plugin="${prefix}/wavy_line_plugin_info.xml" />
  </export>
</package>
```

> pathplanner_wavyline_planner_plugin.launch.py

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Custom Plugin
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'wavyline_planner', 'planner_server.yaml')
    
    # Standard
    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
    default_bt_xml_path = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'behavior.xml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator.yaml')
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
            parameters=[bt_navigator_yaml, {'default_bt_xml_filename': default_bt_xml_path}]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['controller_server',
                                        'planner_server',
                                        'recoveries_server',
                                        'bt_navigator']}])
    ])
```

> wavyline_planner/planner_server.yaml

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True    
    planner_plugins: ["GridBased"]    
    GridBased:
      plugin: WavyLineCustomPlugin
      interpolation_resolution: 0.1

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_footprint
      use_sim_time: True
      robot_radius: 0.3
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True
      obstacle_layer:
          plugin: "nav2_costmap_2d::ObstacleLayer"
          enabled: True
          observation_sources: scan
          scan:
            topic: /scan
            max_obstacle_height: 2.0
            clearing: True
            marking: True
            data_type: "LaserScan"
            raytrace_max_range: 3.0
            raytrace_min_range: 0.0
            obstacle_max_range: 2.5
            obstacle_min_range: 0.0
```

> setup.py

```python
from setuptools import setup
import os
from glob import glob


package_name = 'path_planner_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config/custom_costmap'), glob('config/custom_costmap/*.yaml')),
        (os.path.join('share', package_name, 'config/my_custom_crazy_costmap'), glob('config/my_custom_crazy_costmap/*.yaml')),
        (os.path.join('share', package_name, 'config/straightline_planner'), glob('config/straightline_planner/*.yaml')),
        (os.path.join('share', package_name, 'config/wavyline_planner'), glob('config/wavyline_planner/*.yaml')),
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

## Plug-in do Controlador
E para este último exemplo, crie um plugin personalizado para o planejador local. Existem muitos implementados, mas este é baseado no código Pure Pursuit desenvolvido pelo código original de Steve Macenski. É uma nova visão do controlador local que tenta melhorá-lo. Isso é baseado no papel SOURCE.

Em vez de usar o algoritmo DWB, a ideia é usar essa busca pura baseada em olhar à frente do caminho e gerar uma trajetória com base nisso. Dependendo de quanto você olhar à frente, seguirá o caminho mais ou menos vagamente.

De uma chance. Siga os mesmos passos dos Exemplos 1. e 2. Coloque o código aqui e comente apenas o que for relevante para o plugin controller:

### 1. Criar o pacote

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake custom_nav2_controller_plugin --dependencies rclcpp geometry_msgs nav2_costmap_2d pluginlib nav_msgs nav2_util nav2_core tf2
cd ~/ros2_ws
colcon build --packages-select custom_nav2_controller_plugin
source install/setup.bash
```

### 2: criar o arquivo de origem e o arquivo de cabeçalho

```bash
cd ~/ros2_ws/src/custom_nav2_controller_plugin
touch src/regulated_pure_pursuit_controller.cpp
touch include/custom_nav2_controller_plugin/regulated_pure_pursuit_controller.hpp
```

> regulated_pure_pursuit_controller.cpp
```cpp
// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <utility>

#include "custom_nav2_controller_plugin/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;  // NOLINT

namespace custom_nav2_controller_plugin_namespace_name
{

void RegulatedPurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  double transform_tolerance = 0.1;
  double control_frequency = 20.0;
  goal_dist_tol_ = 0.25;  // reasonable default before first update

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(1.8));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_approach_linear_velocity_scaling", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_allowed_time_to_collision", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_regulated_linear_velocity_scaling", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cost_scaling_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cost_scaling_gain", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".inflation_cost_scaling_factor", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".regulated_linear_scaling_min_radius", rclcpp::ParameterValue(0.90));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".regulated_linear_scaling_min_speed", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_min_angle", rclcpp::ParameterValue(0.785));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(3.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".allow_reversing", rclcpp::ParameterValue(false));

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  base_desired_linear_vel_ = desired_linear_vel_;
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(
    plugin_name_ + ".rotate_to_heading_angular_vel",
    rotate_to_heading_angular_vel_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(
    plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    use_velocity_scaled_lookahead_dist_);
  node->get_parameter(
    plugin_name_ + ".min_approach_linear_velocity",
    min_approach_linear_velocity_);
  node->get_parameter(
    plugin_name_ + ".use_approach_linear_velocity_scaling",
    use_approach_vel_scaling_);
  node->get_parameter(
    plugin_name_ + ".max_allowed_time_to_collision",
    max_allowed_time_to_collision_);
  node->get_parameter(
    plugin_name_ + ".use_regulated_linear_velocity_scaling",
    use_regulated_linear_velocity_scaling_);
  node->get_parameter(
    plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
    use_cost_regulated_linear_velocity_scaling_);
  node->get_parameter(plugin_name_ + ".cost_scaling_dist", cost_scaling_dist_);
  node->get_parameter(plugin_name_ + ".cost_scaling_gain", cost_scaling_gain_);
  node->get_parameter(
    plugin_name_ + ".inflation_cost_scaling_factor",
    inflation_cost_scaling_factor_);
  node->get_parameter(
    plugin_name_ + ".regulated_linear_scaling_min_radius",
    regulated_linear_scaling_min_radius_);
  node->get_parameter(
    plugin_name_ + ".regulated_linear_scaling_min_speed",
    regulated_linear_scaling_min_speed_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
  node->get_parameter(plugin_name_ + ".rotate_to_heading_min_angle", rotate_to_heading_min_angle_);
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(plugin_name_ + ".allow_reversing", allow_reversing_);
  node->get_parameter("controller_frequency", control_frequency);

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  control_duration_ = 1.0 / control_frequency;

  if (inflation_cost_scaling_factor_ <= 0.0) {
    RCLCPP_WARN(
      logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
      "it should be >0. Disabling cost regulated linear velocity scaling.");
    use_cost_regulated_linear_velocity_scaling_ = false;
  }

  /** Possible to drive in reverse direction if and only if
   "use_rotate_to_heading" parameter is set to false **/

  if (use_rotate_to_heading_ && allow_reversing_) {
    RCLCPP_WARN(
      logger_, "Disabling reversing. Both use_rotate_to_heading and allow_reversing "
      "parameter cannot be set to true. By default setting use_rotate_to_heading true");
    allow_reversing_ = false;
  }

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
  carrot_arc_pub_ = node->create_publisher<nav_msgs::msg::Path>("lookahead_collision_arc", 1);
}

void RegulatedPurePursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_.reset();
  carrot_pub_.reset();
  carrot_arc_pub_.reset();
}

void RegulatedPurePursuitController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_->on_activate();
  carrot_pub_->on_activate();
  carrot_arc_pub_->on_activate();
}

void RegulatedPurePursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
  carrot_arc_pub_->on_deactivate();
}

std::unique_ptr<geometry_msgs::msg::PointStamped> RegulatedPurePursuitController::createCarrotMsg(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header = carrot_pose.header;
  carrot_msg->point.x = carrot_pose.pose.position.x;
  carrot_msg->point.y = carrot_pose.pose.position.y;
  carrot_msg->point.z = 0.01;  // publish right over map to stand out
  return carrot_msg;
}

double RegulatedPurePursuitController::getLookAheadDistance(const geometry_msgs::msg::Twist & speed)
{
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance
  double lookahead_dist = lookahead_dist_;
  if (use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = fabs(speed.linear.x) * lookahead_time_;
    lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  }

  return lookahead_dist;
}

geometry_msgs::msg::TwistStamped RegulatedPurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{

  RCLCPP_WARN(logger_, "START computeVelocityCommands >>>>>>>>>>>>>>>>>>>");
  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tol_ = pose_tolerance.position.x;
  }

  // Transform path to robot base frame
  auto transformed_plan = transformGlobalPlan(pose);

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(speed);

  // Check for reverse driving
  if (allow_reversing_) {
    // Cusp check
    double dist_to_direction_change = findDirectionChange(pose);

    // if the lookahead distance is further than the cusp, use the cusp distance instead
    if (dist_to_direction_change < lookahead_dist) {
      lookahead_dist = dist_to_direction_change;
    }
  }

  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  double linear_vel, angular_vel;

  // Find distance^2 to look ahead point (carrot) in robot base frame
  // This is the chord length of the circle
  const double carrot_dist2 =
    (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
    (carrot_pose.pose.position.y * carrot_pose.pose.position.y);

  // Find curvature of the circle (k = 1 / R)
  double curvature = 0.0;
  if (carrot_dist2 > 0.001) {
    curvature = 2.0 * carrot_pose.pose.position.y / carrot_dist2;
  }

  // Setting the velocity direction
  double sign = 1.0;
  if (allow_reversing_) {
    sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
  }

  linear_vel = desired_linear_vel_;

  // Make sure you are in compliance with basic constraints
  double angle_to_heading;
  if (shouldRotateToGoalHeading(carrot_pose)) {
    double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
  } else if (shouldRotateToPath(carrot_pose, angle_to_heading)) {
    rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
  } else {
    applyConstraints(
      fabs(lookahead_dist - sqrt(carrot_dist2)),
      lookahead_dist, curvature, speed,
      costAtPose(pose.pose.position.x, pose.pose.position.y), linear_vel, sign);

    // Apply curvature to angular velocity after constraining linear velocity
    angular_vel = linear_vel * curvature;
  }

  // Collision checking on this velocity heading
  if (isCollisionImminent(pose, linear_vel, angular_vel)) {
    throw nav2_core::PlannerException("RegulatedPurePursuitController detected collision ahead!");
  }

  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
  return cmd_vel;
}

bool RegulatedPurePursuitController::shouldRotateToPath(
  const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path)
{
  // Whether you should rotate robot to rough path heading
  angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  return use_rotate_to_heading_ && fabs(angle_to_path) > rotate_to_heading_min_angle_;
}

bool RegulatedPurePursuitController::shouldRotateToGoalHeading(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  // Whether you should rotate robot to goal heading
  double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  return use_rotate_to_heading_ && dist_to_goal < goal_dist_tol_;
}

void RegulatedPurePursuitController::rotateToHeading(
  double & linear_vel, double & angular_vel,
  const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed)
{
  // Rotate in place using max angular velocity / acceleration possible
  linear_vel = 0.0;
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  angular_vel = sign * rotate_to_heading_angular_vel_;

  const double & dt = control_duration_;
  const double min_feasible_angular_speed = curr_speed.angular.z - max_angular_accel_ * dt;
  const double max_feasible_angular_speed = curr_speed.angular.z + max_angular_accel_ * dt;
  angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
}

geometry_msgs::msg::PoseStamped RegulatedPurePursuitController::getLookAheadPoint(
  const double & lookahead_dist,
  const nav_msgs::msg::Path & transformed_plan)
{
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  }

  return *goal_pose_it;
}

bool RegulatedPurePursuitController::isCollisionImminent(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const double & linear_vel, const double & angular_vel)
{
  // Note(stevemacenski): This may be a bit unusual, but the robot_pose is in
  // odom frame and the carrot_pose are in the robot base frame.

  // check current point is OK
  if (inCollision(robot_pose.pose.position.x, robot_pose.pose.position.y)) {
    return true;
  }

  // visualization messages
  nav_msgs::msg::Path arc_pts_msg;
  arc_pts_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
  arc_pts_msg.header.stamp = robot_pose.header.stamp;
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = arc_pts_msg.header.frame_id;
  pose_msg.header.stamp = arc_pts_msg.header.stamp;

  const double projection_time = costmap_->getResolution() / fabs(linear_vel);

  geometry_msgs::msg::Pose2D curr_pose;
  curr_pose.x = robot_pose.pose.position.x;
  curr_pose.y = robot_pose.pose.position.y;
  curr_pose.theta = tf2::getYaw(robot_pose.pose.orientation);

  int i = 1;
  while (true) {
    // only forward simulate within time requested
    if (i * projection_time > max_allowed_time_to_collision_) {
      break;
    }

    i++;

    // apply velocity at curr_pose over distance
    curr_pose.x += projection_time * (linear_vel * cos(curr_pose.theta));
    curr_pose.y += projection_time * (linear_vel * sin(curr_pose.theta));
    curr_pose.theta += projection_time * angular_vel;

    // store it for visualization
    pose_msg.pose.position.x = curr_pose.x;
    pose_msg.pose.position.y = curr_pose.y;
    pose_msg.pose.position.z = 0.01;
    arc_pts_msg.poses.push_back(pose_msg);

    // check for collision at this point
    if (inCollision(curr_pose.x, curr_pose.y)) {
      carrot_arc_pub_->publish(arc_pts_msg);
      return true;
    }
  }

  carrot_arc_pub_->publish(arc_pts_msg);

  return false;
}

bool RegulatedPurePursuitController::inCollision(const double & x, const double & y)
{
  unsigned int mx, my;

  if (!costmap_->worldToMap(x, y, mx, my)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 30000,
      "The dimensions of the Costmap are too small to successfully check for "
      "collisions as far ahead as requested. Proceed at your own risk, slow the robot, or "
      "increase your Costmap size.");
    return false;
  }

  unsigned char cost = costmap_->getCost(mx, my);

  if (costmap_ros_->getLayeredCostmap()->isTrackingUnknown()) {
    return cost >= INSCRIBED_INFLATED_OBSTACLE && cost != NO_INFORMATION;
  } else {
    return cost >= INSCRIBED_INFLATED_OBSTACLE;
  }
}

double RegulatedPurePursuitController::costAtPose(const double & x, const double & y)
{
  unsigned int mx, my;

  if (!costmap_->worldToMap(x, y, mx, my)) {
    RCLCPP_FATAL(
      logger_,
      "The dimensions of the Costmap are too small to fully include your robot's footprint,"
      "thusly the robot cannot proceed further");
    throw nav2_core::PlannerException(
            "RegulatedPurePursuitController: Dimensions of the Costmap are too small".
            "to encapsulate the robot footprint at current speeds!");
  }

  unsigned char cost = costmap_->getCost(mx, my);
  return static_cast<double>(cost);
}

void RegulatedPurePursuitController::applyConstraints(
  const double & dist_error, const double & lookahead_dist,
  const double & curvature, const geometry_msgs::msg::Twist & /*curr_speed*/,
  const double & pose_cost, double & linear_vel, double & sign)
{
  double curvature_vel = linear_vel;
  double cost_vel = linear_vel;
  double approach_vel = linear_vel;

  // limit the linear velocity by curvature
  const double radius = fabs(1.0 / curvature);
  const double & min_rad = regulated_linear_scaling_min_radius_;
  if (use_regulated_linear_velocity_scaling_ && radius < min_rad) {
    curvature_vel *= 1.0 - (fabs(radius - min_rad) / min_rad);
  }

  // limit the linear velocity by proximity to obstacles
  if (use_cost_regulated_linear_velocity_scaling_ &&
    pose_cost != static_cast<double>(NO_INFORMATION) &&
    pose_cost != static_cast<double>(FREE_SPACE))
  {
    const double inscribed_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
    const double min_distance_to_obstacle = (-1.0 / inflation_cost_scaling_factor_) *
      std::log(pose_cost / (INSCRIBED_INFLATED_OBSTACLE - 1)) + inscribed_radius;

    if (min_distance_to_obstacle < cost_scaling_dist_) {
      cost_vel *= cost_scaling_gain_ * min_distance_to_obstacle / cost_scaling_dist_;
    }
  }

  // Use the lowest of the 2 constraint heuristics, but above the minimum translational speed
  linear_vel = std::min(cost_vel, curvature_vel);
  linear_vel = std::max(linear_vel, regulated_linear_scaling_min_speed_);

  // if the actual lookahead distance is shorter than requested, that means we're at the
  // end of the path. You will scale linear velocity by error to slow to a smooth stop.
  // This expression is eq. to (1) holding time to goal, t, constant using the theoretical
  // lookahead distance and proposed velocity and (2) using t with the actual lookahead
  // distance to scale the velocity (e.g. t = lookahead / velocity, v = carrot / t).
  if (use_approach_vel_scaling_ && dist_error > 2.0 * costmap_->getResolution()) {
    double velocity_scaling = 1.0 - (dist_error / lookahead_dist);
    double unbounded_vel = approach_vel * velocity_scaling;
    if (unbounded_vel < min_approach_linear_velocity_) {
      approach_vel = min_approach_linear_velocity_;
    } else {
      approach_vel *= velocity_scaling;
    }

    // Use the lowest velocity between approach and other constraints, if all overlapping
    linear_vel = std::min(linear_vel, approach_vel);
  }

  // Limit linear velocities to be valid
  linear_vel = std::clamp(fabs(linear_vel), 0.0, desired_linear_vel_);
  linear_vel = sign * linear_vel;
}

void RegulatedPurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void RegulatedPurePursuitController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    desired_linear_vel_ = base_desired_linear_vel_;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      desired_linear_vel_ = base_desired_linear_vel_ * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      desired_linear_vel_ = speed_limit;
    }
  }
}

nav_msgs::msg::Path RegulatedPurePursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let us get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // You will discard points on the plan that are outside the local Costmap
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  const double max_costmap_dim = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
  const double max_transform_dist = max_costmap_dim * costmap->getResolution() / 2.0;

  // First find the closest pose on the path to the robot
  auto transformation_begin =
    nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), global_plan_.poses.end(),
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // Find points definitely outside of the Costmap so you will not transform them.
  auto transformation_end = std::find_if(
    transformation_begin, end(global_plan_.poses),
    [&](const auto & global_plan_pose) {
      return euclidean_distance(robot_pose, global_plan_pose) > max_transform_dist;
    });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that you have already passed so we do not
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  global_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

double RegulatedPurePursuitController::findDirectionChange(
  const geometry_msgs::msg::PoseStamped & pose)
{
  // Iterating through the global path to determine the position of the cusp
  for (unsigned int pose_id = 1; pose_id < global_plan_.poses.size() - 1; ++pose_id) {
    // You have two vectors for the dot product OA and AB. Determining the vectors.
    double oa_x = global_plan_.poses[pose_id].pose.position.x -
      global_plan_.poses[pose_id - 1].pose.position.x;
    double oa_y = global_plan_.poses[pose_id].pose.position.y -
      global_plan_.poses[pose_id - 1].pose.position.y;
    double ab_x = global_plan_.poses[pose_id + 1].pose.position.x -
      global_plan_.poses[pose_id].pose.position.x;
    double ab_y = global_plan_.poses[pose_id + 1].pose.position.y -
      global_plan_.poses[pose_id].pose.position.y;

    /* Checking for the existance of cusp, in the path, using the dot product
    and determine its distance from the robot. If there is no cusp in the path,
    then determine the distance to the goal location. */
    if ( (oa_x * ab_x) + (oa_y * ab_y) < 0.0) {
      auto x = global_plan_.poses[pose_id].pose.position.x - pose.pose.position.x;
      auto y = global_plan_.poses[pose_id].pose.position.y - pose.pose.position.y;
      return hypot(x, y);  // returning the distance if there is a cusp
    }
  }

  return std::numeric_limits<double>::max();
}

bool RegulatedPurePursuitController::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}
}  // namespace custom_nav2_controller_plugin_namespace_name

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  custom_nav2_controller_plugin_namespace_name::RegulatedPurePursuitController,
  nav2_core::Controller)
```

> regulated_pure_pursuit_controller.hpp

```cpp
// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CUSTOM_REGULATED_PURE_PURSUIT_CONTROLLER__HPP_
#define CUSTOM_REGULATED_PURE_PURSUIT_CONTROLLER__HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace custom_nav2_controller_plugin_namespace_name
{

/**
 * @class custom_nav2_controller_plugin_namespace_name::RegulatedPurePursuitController
 * @brief Regulated pure pursuit controller plugin
 */
class RegulatedPurePursuitController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for custom_nav2_controller_plugin_namespace_name::RegulatedPurePursuitController
   */
  RegulatedPurePursuitController() = default;

  /**
   * @brief Destrructor for custom_nav2_controller_plugin_namespace_name::RegulatedPurePursuitController
   */
  ~RegulatedPurePursuitController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of the environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in a false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Transforms global plan into the same frame as pose, clips far away poses, and possibly prunes passed poses
   * @param Pose to transform
   * @return Path in the new frame
   */
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Transform a pose to another frame.
   * @param frame Frame ID to transform to
   * @param in_pose Pose input to transform
   * @param out_pose transformed output
   * @return bool if successful
   */
  bool transformPose(
    const std::string frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  /**
   * @brief Get lookahead distance
   * @param cmd the current speed to use to compute lookahead point
   * @return lookahead distance
   */
  double getLookAheadDistance(const geometry_msgs::msg::Twist &);

  /**
   * @brief Creates a PointStamped message for visualization
   * @param carrot_pose Input carrot point as a PoseStamped
   * @return CarrotMsg a carrot point marker, PointStamped
   */
  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
    const geometry_msgs::msg::PoseStamped & carrot_pose);

  /**
   * @brief Whether the robot should rotate to the rough path heading
   * @param carrot_pose current lookahead point
   * @param angle_to_path Angle of robot output relative to the carrot marker
   * @return Whether should rotate to the path heading
   */
  bool shouldRotateToPath(
    const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path);

  /**
   * @brief Whether robot should rotate to final goal orientation
   * @param carrot_pose current lookahead point
   * @return Whether should rotate to goal heading
   */
  bool shouldRotateToGoalHeading(const geometry_msgs::msg::PoseStamped & carrot_pose);

  /**
   * @brief Create a smooth and kinematically smoothed rotation command
   * @param linear_vel linear velocity
   * @param angular_vel angular velocity
   * @param angle_to_path Angle of robot output relative to the carrot marker
   * @param curr_speed the current robot speed
   */
  void rotateToHeading(
    double & linear_vel, double & angular_vel,
    const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed);

  /**
   * @brief Whether a collision is imminent
   * @param robot_pose Pose of robot
   * @param carrot_pose Pose of carrot
   * @param linear_vel linear velocity to forward project
   * @param angular_vel angular velocity to forward project
   * @return Whether collision is imminent
   */
  bool isCollisionImminent(
    const geometry_msgs::msg::PoseStamped &,
    const double &, const double &);

  /**
   * @brief Whether the point is in collision
   * @param x Pose of pose x
   * @param y Pose of pose y
   * @param theta orientation of Yaw
   * @return Whether in collision
   */
  bool inCollision(const double & x, const double & y);

  /**
   * @brief Cost at a point
   * @param x Pose of pose x
   * @param y Pose of pose y
   * @return Cost of pose in Costmap
   */
  double costAtPose(const double & x, const double & y);

  /**
   * @brief apply regulation constraints to the system
   * @param linear_vel robot command linear velocity input
   * @param dist_error error in the carrot distance and lookahead distance
   * @param lookahead_dist optimal lookahead distance
   * @param curvature of the path
   * @param speed of robot
   * @param pose_cost cost at this pose
   */
  void applyConstraints(
    const double & dist_error, const double & lookahead_dist,
    const double & curvature, const geometry_msgs::msg::Twist & speed,
    const double & pose_cost, double & linear_vel, double & sign);

  /**
   * @brief get a lookahead point
   * @param lookahead_dist Optimal lookahead distance
   * @param path Current global path
   * @return lookahead point
   */
  geometry_msgs::msg::PoseStamped getLookAheadPoint(const double &, const nav_msgs::msg::Path &);

  /**
   * @brief checks for the cusp position
   * @param pose Pose input to determine the cusp position
   * @return robot distance from the cusp
   */
  double findDirectionChange(const geometry_msgs::msg::PoseStamped & pose);

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_ {rclcpp::get_logger("RegulatedPurePursuitController")};
  rclcpp::Clock::SharedPtr clock_;

  double desired_linear_vel_, base_desired_linear_vel_;
  double lookahead_dist_;
  double rotate_to_heading_angular_vel_;
  double max_lookahead_dist_;
  double min_lookahead_dist_;
  double lookahead_time_;
  bool use_velocity_scaled_lookahead_dist_;
  tf2::Duration transform_tolerance_;
  bool use_approach_vel_scaling_;
  double min_approach_linear_velocity_;
  double control_duration_;
  double max_allowed_time_to_collision_;
  bool use_regulated_linear_velocity_scaling_;
  bool use_cost_regulated_linear_velocity_scaling_;
  double cost_scaling_dist_;
  double cost_scaling_gain_;
  double inflation_cost_scaling_factor_;
  double regulated_linear_scaling_min_radius_;
  double regulated_linear_scaling_min_speed_;
  bool use_rotate_to_heading_;
  double max_angular_accel_;
  double rotate_to_heading_min_angle_;
  double goal_dist_tol_;
  bool allow_reversing_;

  nav_msgs::msg::Path global_plan_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>>
  carrot_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> carrot_arc_pub_;
};

}  // namespace custom_nav2_controller_plugin_namespace_name

#endif  // CUSTOM_REGULATED_PURE_PURSUIT_CONTROLLER__HPP_
```

Como antes, esta classe é baseada, neste caso, em `nav2_core::Controller`. Essa classe base possui vários métodos que precisam estar dentro do seu plugin para funcionar plug and play. Todos eles são OBRIGATÓRIOS e alguns são iguais ao `nav2_core::GlobalPlanner`.

* configure(): OBRIGATÓRIO.
* activate(): OBRIGATÓRIO.
* deactivate(): OBRIGATÓRIO.
* cleanup(): OBRIGATÓRIO.
* setPlan(): OBRIGATÓRIO. Chame-o quando o plano global for atualizado.
* computeVelocityCommands(): OBRIGATÓRIO. Gera os comandos de velocidade enviados ao robô para seguir o plano global gerado no plugin global path planner. Ele retorna um `geometry_msgs::msg::TwistStamped` usado para mover o robô. Tem como parâmetros a pose atual do robô e a velocidade atual.

### 3: criar o arquivo de informações do plug-in.xml
Executar no Shell
```bash
cd ~/ros2_ws/src/custom_nav2_controller_plugin
touch regulated_pure_pursuit_controller_info.xml
```

> regulated_pure_pursuit_controller_info.xml

```xml
<class_libraries>
    <library path="custom_pursuit_controller_plugin_core">
        <class type="custom_nav2_controller_plugin_namespace_name::RegulatedPurePursuitController" base_class_type="nav2_core::Controller">
            <description>
                Pure Pursuit Controller
            </description>
        </class>
    </library>
</class_libraries>
```

Observe que isso é um pouco diferente. Pode ser feito desta forma também. Observe que:

Você tem uma class_libraries extra. Isso está nas versões mais recentes do ROS2 e é recomendado.
Não há `< nome da classe>`. Isso significa que ao adicionar este plug-in ao arquivo `.yaml`, o nome é o mesmo do tipo de classe. Nesse caso, `custom_nav2_controller_plugin_namespace_name::RegulatedPurePursuitController`.

### 4: configurar o CMakelists.txt e o package.xml para compilação

> CMakelists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(custom_nav2_controller_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(custom_nav2_controller_plugin_name custom_pursuit_controller_plugin_core)

include_directories(
  include
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(tf2 REQUIRED)


add_library(${custom_nav2_controller_plugin_name} SHARED
        src/regulated_pure_pursuit_controller.cpp)

# prevent pluginlib from using boost
target_compile_definitions(${custom_nav2_controller_plugin_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(${custom_nav2_controller_plugin_name}
  ${dependencies}
)


install(TARGETS ${custom_nav2_controller_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${custom_nav2_controller_plugin_name})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core regulated_pure_pursuit_controller_info.xml)

ament_package()
```

> package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>custom_nav2_controller_plugin</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="duckfrost@gmail.com">tgrip</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  <depend>nav2_costmap_2d</depend>
  <depend>pluginlib</depend>
  <depend>nav_msgs</depend>
  <depend>nav2_util</depend>
  <depend>nav2_core</depend>
  <depend>tf2</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
    <nav2_core plugin="${prefix}/regulated_pure_pursuit_controller_info.xml" />
  </export>
</package>
```

### 5: Configurar, compilar e testar

#### Configurar

```bash
cd ~/ros2_ws/
touch ~/ros2_ws/src/neobotix_mp_400_navigation/path_planner_server/launch/pathplanner_purepursuit_controller_plugin.launch.py
mkdir ~/ros2_ws/src/neobotix_mp_400_navigation/path_planner_server/config/pure_pursuit_controller
cd ~/ros2_ws/src/neobotix_mp_400_navigation/path_planner_server/config/pure_pursuit_controller
touch controller.yaml
```

#### controller.yaml
```yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 5.0
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
    # Custom Pure Pusuit Params
    FollowPath:
      plugin: "custom_nav2_controller_plugin_namespace_name::RegulatedPurePursuitController"
      desired_linear_vel: 2.5
      max_linear_accel: 2.5
      max_linear_decel: 2.5
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 1.8
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: false
      min_approach_linear_velocity: 0.05
      use_approach_linear_velocity_scaling: true
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: false
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.25
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.785
      max_angular_accel: 3.2
      cost_scaling_dist: 0.3
      cost_scaling_gain: 1.0
      inflation_cost_scaling_factor: 3.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_footprint
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.4
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        map_subscribe_transient_local: True
      always_send_full_costmap: True
```

Como você pode ver, este plugin tem muitos PARÂMETROS. Mas os mais importantes afetarão rapidamente o comportamento são:

* `desired_linear_vel`: Isso faz com que o robô se mova muito rápido. Nesse caso, está definido para 2,5 metros por segundo, o que é rápido.
* `lookahead_dist`: Regula até que distância você verifica colisões no caminho. Quanto mais alto, mais suaves serão as trajetórias, mas menos precisas ao seguir o caminho. É limitado pelos parâmetros `min_lookahead_dist` e `max_lookahead_dist`.

```yaml
controller_plugins: ["FollowPath"]
...
FollowPath:
    plugin: "custom_nav2_controller_plugin_namespace_name::RegulatedPurePursuitController"
    desired_linear_vel: 2.5
    max_linear_accel: 2.5
    max_linear_decel: 2.5
    lookahead_dist: 0.6
    min_lookahead_dist: 0.3
    max_lookahead_dist: 0.9
    lookahead_time: 1.5
    rotate_to_heading_angular_vel: 1.8
    transform_tolerance: 0.1
    use_velocity_scaled_lookahead_dist: false
    min_approach_linear_velocity: 0.05
    use_approach_linear_velocity_scaling: true
    max_allowed_time_to_collision_up_to_carrot: 1.0
    use_regulated_linear_velocity_scaling: true
    use_cost_regulated_linear_velocity_scaling: false
    regulated_linear_scaling_min_radius: 0.9
    regulated_linear_scaling_min_speed: 0.25
    use_rotate_to_heading: true
    rotate_to_heading_min_angle: 0.785
    max_angular_accel: 3.2
    cost_scaling_dist: 0.3
    cost_scaling_gain: 1.0
    inflation_cost_scaling_factor: 3.0
```

> pathplanner_purepursuit_controller_plugin.launch.py

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Custom Plugin
    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'pure_pursuit_controller', 'controller.yaml')

    # Default
    default_bt_xml_path = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'behavior.xml')
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
            parameters=[bt_navigator_yaml, {'default_bt_xml_filename': default_bt_xml_path}]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['controller_server',
                                        'planner_server',
                                        'recoveries_server',
                                        'bt_navigator']}])
    ])
```

> setup.py

```python
from setuptools import setup
import os
from glob import glob


package_name = 'path_planner_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config/custom_costmap'), glob('config/custom_costmap/*.yaml')),
        (os.path.join('share', package_name, 'config/my_custom_crazy_costmap'), glob('config/my_custom_crazy_costmap/*.yaml')),
        (os.path.join('share', package_name, 'config/straightline_planner'), glob('config/straightline_planner/*.yaml')),
        (os.path.join('share', package_name, 'config/wavyline_planner'), glob('config/wavyline_planner/*.yaml')),
        (os.path.join('share', package_name, 'config/pure_pursuit_controller'), glob('config/pure_pursuit_controller/*.yaml')),
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

#### Compilar

```bash
cd ~/ros2_ws/
colcon build --packages-select custom_nav2_controller_plugin path_planner_server
source install/setup.bash
```

#### Iniciar localização

Esse é o mesmo:
```bash
cd ~/ros2_ws
source install/setup.bash;reset;ros2 launch localization_server localization.launch.py
```

#### Iniciar pathplanner

```bash
cd ~/ros2_ws
source install/setup.bash;reset;ros2 launch path_planner_server pathplanner_purepursuit_controller_plugin.launch.py
```

Agora você deve acessar as Ferramentas Gráficas (deve abrir automaticamente) e alterar o arquivo de configuração do RVIZ para o planejamento do caminho. Deve estar no caminho `~/ros2_ws/src/neobotix_mp_400_navigation/path_planner_server/rviz_config/pathplanning.rviz`.

Vá para `File->Open` Config e selecione esse arquivo.

Agora você deve ver algo assim:

Você pode ver que o robô se move MUITO RÁPIDO. Isso ocorre porque você definiu a alta velocidade de 2,5 m/s.