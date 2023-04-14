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

Como você pode ver, é um tópico profundo e seria necessário um curso inteiro para explicar detalhadamente os plugins.

Agora você criará vários exemplos e exercícios que ensinam como modificar e criar Plugins CUSTOM para **Costmaps**, **Planning** e **Controllers**.

## Criação de plug-ins Nav2 personalizados

A maioria das etapas para criar os plug-ins são idênticas, mas existem alguns elementos diferentes baseados essencialmente na classe Base na qual cada plug-in pode ser baseado.

Você também precisa de uma pilha de navegação funcional para testar todos os novos plugins. Para isso, configure uma navegação padrão pronta para uso, caso não tenha conseguido fazê-la funcionar nas unidades anteriores ou tenha pulado para esta seção e não tenha concluído as unidades anteriores.

### Configuração de navegação

Se você deseja que a localização comece corretamente e não precise mover o robô para relocalizar, você tem duas opções:

1. Inicie a localização desde o início. Não o interrompa durante sua sessão nesta unidade.
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