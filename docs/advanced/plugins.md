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