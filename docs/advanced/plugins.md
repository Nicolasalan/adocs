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
