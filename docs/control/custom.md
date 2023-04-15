# Criando um controlador personalizado
Esta unidade descreve como escrever um novo controlador personalizado mínimo que receba sinais de entrada e gere os sinais de controle enviados aos atuadores usando a estrutura `ros2_control`.

## Introdução
Os controladores estão no coração do sistema ros2_control. Conforme abordado anteriormente na unidade 6, o pacote ros2_controllers inclui um conjunto de controladores padrão comuns e abrangentes que atendem a muitos casos de uso comuns. Exemplos de tais controladores são controlador de esforço, controlador de trajetória e controlador de acionamento diferencial e a lista de controladores fornecidos está crescendo a cada versão do ROS2. No entanto, tanto quanto alguns controladores podem ser amplamente aplicados a muitos tipos diferentes de aplicações robóticas, também é verdade que algumas aplicações também requerem controladores extremamente específicos, dependendo da natureza e requisitos de uma aplicação.

Felizmente, o `ros2_control` é flexível o suficiente para permitir que você crie controladores personalizados para estender sua funcionalidade. Dessa forma, você também pode usar `ros2_control` para resolver problemas de controle mais complexos sem muito trabalho. Nesta unidade do curso, você aprenderá como criar um controlador personalizado seguindo algumas etapas simples. Apenas um breve lembrete: tenha em mente que o foco deste curso é o framework `ros2_control` e não sobre a teoria de controle de robôs. Para uma discussão sobre os conceitos teóricos subjacentes dos controladores, consulte o curso de teoria de controle de robôs. Hora de colocar a mão na massa em controladores personalizados!

## Seu controlador personalizado em cinco etapas
Para implementar uma interface de hardware para um dispositivo de hardware, você precisará fazer o seguinte:

* Crie um pacote para seu controlador personalizado
* Escreva um arquivo de cabeçalho
* Adicionar um arquivo de origem `.cpp`
* Prepare os arquivos `CMakeLists.txt` e `package.xml` para compilação
* Registre seu controlador como um plug-in

## Crie um pacote para seu controlador personalizado
Como de costume, a abordagem recomendada é configurar um novo pacote para manter nosso novo controlador modular e intercambiável.

Primeiro, vá para o diretório src dentro de `ros2_ws`:
```bash
cd ~/ros2_ws/src
```
Agora crie o novo pacote. Este novo pacote deve ter ament_cmake como um tipo de compilação e, como prática recomendada, sugerimos terminar o nome do novo pacote com `_controller`. Você também precisará de algumas dependências:

* control_msgs
* controller_interface
* hardware_interface
* pluginlib
* rclcpp
* rclcpp_lifecycle
* realtime_tools
* exemplo_interfaces

```bash
ros2 pkg create --build-type=ament_cmake rrbot_controller --dependencies control_msgs controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools example_interfaces
```
Seu diretório de pacotes agora deve ter um `CMakeLists.txt` e um arquivo `package.xml` e um src e um diretório `include/rrbot_controller`.

## Add a header file (.hpp)

Agora, dentro da pasta `include/rrbot_controller`, crie um arquivo chamado `rrbot_controller.hpp`.
```bash
touch ~/ros2_ws/src/rrbot_controller/include/rrbot_controller/rrbot_controller.hpp
```
Em seguida, adicione o seguinte trecho de código nesse arquivo:
>  rrbot_controller.hpp
```cpp
// Copyright (c) 2021, Bence Magyar and Denis Stogl
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

#ifndef RRBOT_CONTROLLER__RRBOT_CONTROLLER_HPP_
#define RRBOT_CONTROLLER__RRBOT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"

namespace rrbot_controller {
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RRBotController : public controller_interface::ControllerInterface {
public:
  RRBotController();

  CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

protected:
  std::vector<std::string> joint_names_;
  std::string interface_name_;

  // Command subscribers and Controller State publisher
  using ControllerCommandMsg = control_msgs::msg::JointJog;

  rclcpp::Subscription<ControllerCommandMsg>::SharedPtr command_subscriber_ =
      nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerCommandMsg>>
      input_command_;

  using ControllerStateMsg = control_msgs::msg::JointControllerState;
  using ControllerStatePublisher =
      realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;
};

} // namespace rrbot_controller

#endif // RRBOT_CONTROLLER__RRBOT_CONTROLLER_HPP_
```
#### O código explicado
Ok, vamos dividi-lo e descrever o que cada linha faz.
```cpp
#ifndef RRBOT_CONTROLLER__RRBOT_CONTROLLER_HPP_
#define RRBOT_CONTROLLER__RRBOT_CONTROLLER_HPP_
```
Aqui, adicione protetores de cabeçalho ao arquivo de cabeçalho para garantir que ele não seja incluído mais de uma vez.
```cpp
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
// #include "rrbot_controller/visibility_control.h"

#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"
```
Aqui, use as diretivas `#include` para incluir o conteúdo dos arquivos especificados no compilador.
```cpp
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
```
Isso nos permitirá usar o formato mais curto `CallbackReturn::SUCCESS` e `CallbackReturn::ERROR` em vez do nome de namespace totalmente qualificado  `rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS` posteriormente no código.
```cpp
namespace rrbot_controller
```
Isso é para colocar nossa implementação em um namespace que corresponda ao nome do pacote.
```cpp
class RRBotController : public controller_interface::ControllerInterface
```
Aqui estamos apenas declarando a classe `RRBotController`, que herda a classe `controller_interface::ControllerInterface`. A classe posterior é a classe da qual todo controlador personalizado deve herdar para ser construído corretamente.
```cpp
public:
  RRBotController();

  CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
```
Depois, devemos declarar esses métodos públicos que são aqueles que qualquer controlador personalizado deve implementar ou substituir no arquivo .cpp.

Explicaremos cada um desses métodos no momento em que os implementarmos no código.
```cpp
protected:
  std::vector<std::string> joint_names_;
  std::string interface_name_;

  // Command subscribers and Controller State publisher
  using ControllerCommandMsg = control_msgs::msg::JointJog;

  rclcpp::Subscription<ControllerCommandMsg>::SharedPtr command_subscriber_ =
      nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerCommandMsg>>
      input_command_;

  using ControllerStateMsg = control_msgs::msg::JointControllerState;
  using ControllerStatePublisher =
      realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_
```
Por fim, declaramos as variáveis que precisaremos para este controlador, bem como um assinante e um objeto publicador.
> A forma como esses métodos são nomeados passou por algumas mudanças nas últimas versões do ROS2_Control. Este curso é para a versão ROS2 Galactic. Embora as mudanças sejam pequenas, é importante para você saber qual versão do ROS2_Control você possui.

## Add a .cpp source file
Em seguida, devemos adicionar o arquivo de origem correspondente dentro da pasta src do pacote e definir todos os métodos que acabamos de declarar no arquivo de cabeçalho.

Dentro da pasta `src`, crie um arquivo chamado `rrbot_controller.cpp`.
```bash
touch ~/ros2_ws/src/rrbot_controller/src/rrbot_controller.cpp
```
Em seguida, adicione o seguinte trecho de código nesse arquivo:

> rrbot_controller.cpp
```cpp
// Copyright (c) 2021, Bence Magyar and Denis Stogl
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

#include "rrbot_controller/rrbot_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace rrbot_controller
{
RRBotController::RRBotController() : controller_interface::ControllerInterface() {}
    
// All the methods added must be inside this namespace
    
}  // namespace rrbot_controller
```
Salve seu trabalho antes de fazer qualquer outra alteração.

#### O código explicado
Em vez de implementar todos os métodos que pertencem à classe RRBotController de uma só vez, começaremos com os imports e o namespace que encerra esta classe e adicionaremos gradualmente o código:
```cpp
#include "rrbot_controller/rrbot_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>
```
Primeiro, inclua os arquivos de cabeçalho necessários como de costume.
```cpp
namespace rrbot_controller
{
RRBotController::RRBotController() : controller_interface::ControllerInterface() {}
    
// All the methods added must be inside this namespace
    
}  // namespace rrbot_controller
```
Em seguida, adicionamos o chapéu de definição de namespace que inclui a implementação dos métodos da classe RRBotController.
Observe que todos os métodos que adicionaremos a este arquivo devem ser colocados dentro deste namespace.

Na linha de código abaixo, chamamos o construtor padrão da classe base implicitamente dentro da lista de inicializadores do construtor da subclasse.

Esse último colchete de fechamento indica o final do namespace.

## Escreva o método `on_init()`
Durante o estágio inicial, declaramos todos os parâmetros que aceitaremos durante a vida útil do controlador. Isso para que o tipo e o nome do parâmetro fiquem bem definidos na hora da inicialização, o que reduz as chances de configuração incorreta posteriormente.

Para continuar com o exercício da seção anterior, copie e cole o código mostrado abaixo no arquivo `rrbot_controller.cpp`. Certifique-se de colar isso dentro dos colchetes do namespace.

> rrbot_controller.cpp

```cpp
CallbackReturn RRBotController::on_init() {
  try {
    auto_declare("joints", std::vector<std::string>());
    auto_declare("interface_name", std::string());
  } catch (const std::exception &e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n",
            e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}
```
#### O código explicado
```cpp
  try {
    auto_declare("joints", std::vector<std::string>());
    auto_declare("interface_name", std::string());
  } catch (const std::exception &e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n",
            e.what());
    return CallbackReturn::ERROR;
  }
```
Envolvemos o código com um bloco `try/catch` para poder notificar o gerenciador do controlador se algo der errado ao processar o método `on_init()`. Neste exemplo, `on_init()` executa a declaração dos parâmetros que este controlador irá requerer e lê do arquivo de configuração do controlador. Se algo estiver errado com esse segmento de código, esse método retornará um erro e a máquina de estado do gerenciador do controlador fará a transição para `finalizado`.
```cpp
  return CallbackReturn::SUCCESS;
```
Se tudo correr bem, retornamos `CallbackReturn::SUCCESS`.

## Adicione o método on_configure()

O método `on_configure()` é usado para ler os valores de parâmetro e declarar quaisquer assinantes e editores necessários conforme necessário. Esse método é executado antes da chamada do método de atualização e garante que, uma vez que o algoritmo de controle seja colocado em ação, tudo esteja definido e pronto para funcionar.

Aqui está um trecho de código C++ que implementa um modelo de código para `on_configure()`, adicione-o ao seu arquivo de código-fonte.

> rrbot_controller.cpp

```cpp
CallbackReturn RRBotController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  auto error_if_empty = [&](const auto &parameter, const char *parameter_name) {
    if (parameter.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "'%s' parameter was empty",
                   parameter_name);
      return true;
    }
    return false;
  };

  auto get_string_array_param_and_error_if_empty =
      [&](std::vector<std::string> &parameter, const char *parameter_name) {
        parameter = get_node()->get_parameter(parameter_name).as_string_array();
        return error_if_empty(parameter, parameter_name);
      };

  auto get_string_param_and_error_if_empty =
  [&](std::string &parameter, const char *parameter_name) {
    parameter = get_node()->get_parameter(parameter_name).as_string();
    return error_if_empty(parameter, parameter_name);
  };

  if (
    get_string_array_param_and_error_if_empty(joint_names_, "joints") ||
      get_string_param_and_error_if_empty(interface_name_, "interface_name")) {
    return CallbackReturn::ERROR;
  }

  // Command Subscriber and callbacks
  auto callback_command =
      [&](const std::shared_ptr<ControllerCommandMsg> msg) -> void {
    if (msg->joint_names.size() == joint_names_.size()) {
      input_command_.writeFromNonRT(msg);
    } else {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Received %zu , but expected %zu joints in command. "
                   "Ignoring message.",
                   msg->joint_names.size(), joint_names_.size());
    }
  };
  command_subscriber_ = get_node()->create_subscription<ControllerCommandMsg>(
      "~/commands", rclcpp::SystemDefaultsQoS(), callback_command);

  // State publisher
  s_publisher_ =
  get_node()->create_publisher<ControllerStateMsg>(
      "~/state", rclcpp::SystemDefaultsQoS());
  state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);

  state_publisher_->lock();
  state_publisher_->msg_.header.frame_id = joint_names_[0];
  state_publisher_->unlock();

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}
```
#### O código explicado
```cpp
auto error_if_empty = [&](const auto & parameter, const char * parameter_name) {
    if (parameter.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "'%s' parameter was empty", parameter_name);
      return true;
    }
    return false;
  };
```
Na primeira parte do método `on_configure()` declaramos uma função lambda para verificar se há parâmetros vazios e retorna um valor booleano.
```cpp
  auto get_string_array_param_and_error_if_empty =
    [&](std::vector<std::string> & parameter, const char * parameter_name) {
      parameter = get_node()->get_parameter(parameter_name).as_string_array();
      return error_if_empty(parameter, parameter_name);
    };
```
Em seguida, declaramos uma segunda função lambda para recuperar uma string de parâmetro e testamos se essa matriz de parâmetros está vazia usando a primeira função lambda declarada anteriormente.
```cpp
  auto get_string_param_and_error_if_empty =
    [&](std::string & parameter, const char * parameter_name) {
      parameter = get_node()->get_parameter(parameter_name).as_string();
      return error_if_empty(parameter, parameter_name);
    };
```
Depois disso, declaramos uma terceira função lambda para recuperar um parâmetro de string e fazer a verificação vazia que também retorna um valor booleano.
```cpp
  if (
    get_string_array_param_and_error_if_empty(joint_names_, "joints") ||
    get_string_param_and_error_if_empty(interface_name_, "interface_name")) {
    return CallbackReturn::ERROR;
  }
```
É apenas nas poucas linhas de código mostradas acima que chamamos a segunda e a terceira funções lambda para avaliar se a matriz de parâmetros "juntas" e o parâmetro "interface" estão vazios, caso em que o método `on_configure()` retorna `CallbackReturn::ERROR`.

```cpp
  // Command Subscriber and callbacks
  auto callback_command = [&](const std::shared_ptr<ControllerCommandMsg> msg) -> void {
    if (msg->joint_names.size() == joint_names_.size()) {
      input_command_.writeFromNonRT(msg);
    } else {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Received %zu , but expected %zu joints in command. Ignoring message.",
        msg->joint_names.size(), joint_names_.size());
    }
  };
  command_subscriber_ = get_node()->create_subscription<ControllerCommandMsg>(
    "~/commands", rclcpp::SystemDefaultsQoS(), callback_command);
```
Na segunda parte do `on_configure()` mostrado acima, declaramos uma lambada como função de retorno de chamada de comando e declaramos um objeto assinante de comando.
```cpp
  // State publisher
  s_publisher_ =
    get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
  state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);

  state_publisher_->lock();
  state_publisher_->msg_.header.frame_id = joint_names_[0];
  state_publisher_->unlock();
```
Mais abaixo, declaramos um editor para transmitir os estados conjuntos e o tornamos um ponteiro exclusivo. As últimas linhas da árvore acima são usadas para evitar que vários encadeamentos acessem a mensagem publicada ao mesmo tempo ao definir o valor `frame_id` do cabeçalho.
```cpp
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
```
Ao fechar, o método `on_configure()` informa ao usuário sobre sua conclusão bem-sucedida e retorna `CallbackReturn::SUCCESS`.

## Adicione o método `command_interface_configuration()`
Este método é onde definimos quais interfaces de comando são necessárias.

Adicione o seguinte bloco de código em seu arquivo rrbot_controller.cpp logo abaixo do método anterior, mas ainda dentro dos colchetes de namespace.

> rrbot_controller.cpp
```cpp
controller_interface::InterfaceConfiguration RRBotController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(joint_names_.size());
  for (const auto & joint : joint_names_) {
    command_interfaces_config.names.push_back(joint + "/" + interface_name_);
  }

  return command_interfaces_config;
}
```
#### O código explicado
```cpp
controller_interface::InterfaceConfiguration command_interfaces_config;
command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
```
Aqui, primeiro criamos um novo objeto InterfaceConfiguration e definimos o tipo como INDIVIDUAL. Existem três opções de configuração de interface `ALL`, `INDIVIDUAL` e `NONE`. `ALL` e `NONE` solicitarão acesso a todas as interfaces disponíveis ou a nenhuma delas. A configuração `INDIVIDUAL` precisa de uma lista detalhada de nomes de interface necessários. Esses geralmente são fornecidos como parâmetros.

A chamada acima reservará espaço de memória para o vetor de nomes de interface.
```cpp
  command_interfaces_config.names.reserve(joint_names_.size());
  for (const auto & joint : joint_names_) {
    command_interfaces_config.names.push_back(joint + "/" + interface_name_);
  }
```
No bloco acima, a primeira linha reservará espaço de memória para o vetor de nomes de interface.

Em seguida, cada articulação recebe seu próprio nome de interface, que é mantido dentro de `command_interfaces_config.names`. Um nome de interface completo deve ter a estrutura `< joint_name>/< interface_type>`.

## Crie o método `state_interface_configuration()`
Este método executa uma função semelhante ao método anterior, com a diferença de que este método é usado para definir quais interfaces de sensores de hardware são requeridas pelo controlador.

O código a ser adicionado ao seu arquivo `rrbot_controller.cpp` é o seguinte.
```cpp
controller_interface::InterfaceConfiguration RRBotController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(joint_names_.size());
  for (const auto & joint : joint_names_) {
    state_interfaces_config.names.push_back(joint + "/" + interface_name_);
  }

  return state_interfaces_config;
}
```
#### O código explicado
```cpp
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
```
Aqui, mais uma vez, criamos um novo objeto `InterfaceConfiguration` e definimos o tipo como INDIVIDUAL.
```cpp
state_interfaces_config.names.reserve(joint_names_.size());
```
Então reservamos memória para o tamanho da interface.
```cpp
  for (const auto & joint : joint_names_) {
    state_interfaces_config.names.push_back(joint + "/" + interface_name_);
  }
```
Por fim, salvamos os nomes das interfaces dos sensores no objeto `InterfaceConfiguration` criado anteriormente.
Este é um procedimento padrão para armazenar nomes de juntas de arquivos de configuração `.yaml`, portanto, não espere que esse método mude muito em qualquer outra implementação de controlador personalizado.

## Implemente a função de modelo `get_ordered_interfaces()`

Prossiga para adicionar a função de modelo `get_ordered_interfaces()` mostrada abaixo no final do método `state_interface_configuration()` que você adicionou anteriormente.

> rrbot_controller.cpp
```cpp
template <typename T>
bool get_ordered_interfaces(
  std::vector<T> & unordered_interfaces, const std::vector<std::string> & joint_names,
  const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
{
  for (const auto & joint_name : joint_names) {
    for (auto & command_interface : unordered_interfaces) {
      if (
        (command_interface.get_name() == joint_name) &&
        (command_interface.get_interface_name() == interface_type)) {
        ordered_interfaces.push_back(std::ref(command_interface));
      }
    }
  }

  return joint_names.size() == ordered_interfaces.size();
}
```
#### O código explicado
Esta função modelo recebe um vetor não ordenado de interfaces como argumento e o converte em um vetor ordenado. É necessário preencher `orders_interfaces` com referências às interfaces correspondentes na mesma ordem que em `joint_names`.

## Escreva o método `on_activate()`
Usamos o método `on_activate()` para declarar uma mensagem de comando e definir o valor padrão para o comando.

> rrbot_controller.cpp
```cpp
CallbackReturn RRBotController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command
  std::shared_ptr<ControllerCommandMsg> msg = std::make_shared<ControllerCommandMsg>();
  msg->joint_names = joint_names_;
  msg->displacements.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  input_command_.writeFromNonRT(msg);

  return CallbackReturn::SUCCESS;
}
```
#### O código explicado
```cpp
  // Set default value in command
  std::shared_ptr<ControllerCommandMsg> msg = std::make_shared<ControllerCommandMsg>();
  msg->joint_names = joint_names_;
  msg->displacements.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
```
Acima, declaramos uma mensagem usando o tipo de dados `ControllerCommandMsg`, preenchemos o campo `joint_names` na mensagem e definimos o valor padrão no comando para o valor especial `quiet not-a-number`, que é significativo para tipos de ponto flutuante.
```cpp
  input_command_.writeFromNonRT(msg);
```
Aqui usamos `writeFromNonRT` que pode ser usado em tempo real (RT), se tivermos a garantia de que:

* nenhum `thread` não `rt` está chamando a mesma função (não estamos assinando callbacks ros).
* há apenas um único tópico `rt`.

## Escreva o método `on_deactivate()`
O método `on_deactivate()` é chamado pelo gerenciador do controlador para desligar o controlador. Freqüentemente, ele contém apenas uma instrução de retorno que indica sucesso. Se você precisar executar algumas instruções de código para poder executar um desligamento normal, certifique-se de fazê-lo o mais em tempo real possível.

Aqui está um esboço de código C++ que implementa o método `on_deactivate`:
> rrbot_controller.cpp
```cpp

CallbackReturn RRBotController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}
```
#### O código explicado
Não há muito o que fazer aqui além de retornar `Callback Return::SUCCESS` quando o método é chamado.

## Implemente o método update()
A função `update()` é chamada no loop de controle para produzir um comando de controle para o hardware.

Para implementar o método `update()`, cole o código a seguir no final do arquivo que você ainda deve ter aberto na seção anterior.

> rrbot_controller.cpp
```cpp
controller_interface::return_type
RRBotController::update(const rclcpp::Time &time,
                        const rclcpp::Duration & /*period*/) {
  auto current_command = input_command_.readFromRT();

  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    if (!std::isnan((*current_command)->displacements[i])) {
      command_interfaces_[i].set_value((*current_command)->displacements[i]);
    }
  }

  if (state_publisher_ && state_publisher_->trylock()) {
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->msg_.set_point = command_interfaces_[0].get_value();

    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}
```
#### O código explicado
```cpp
auto current_command = input_command_.readFromRT();
```
Aqui estamos obtendo um ponteiro de dados com `readFromRT()`.
```cpp
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    if (!std::isnan((*current_command)->displacements[i])) {
      command_interfaces_[i].set_value((*current_command)->displacements[i]);
    }
  }
```
Para cada junta definimos como valor comandado os dados não modificados que estão localizados dentro do vetor de deslocamentos.
```cpp
  if (state_publisher_ && state_publisher_->trylock()) {
    state_publisher_->msg_.header.stamp = get_node()->now();
    state_publisher_->msg_.set_point = command_interfaces_[0].get_value();

    state_publisher_->unlockAndPublish();
  }
```
Este exemplo está implementando um controlador de comando Forward para um conjunto de juntas. Basicamente, ele encaminha o sinal de comando para um conjunto de juntas.
## Adicione a macro PLUGINLIB_EXPORT_CLASS
Depois que o namespace for fechado, adicione uma chamada à macro `PLUGINLIB_EXPORT_CLASS` no final do arquivo `.cpp`.
Adicione o seguinte bloco de código na parte inferior do arquivo de código-fonte, depois que o identificador de namespace for fechado:

> rrbot_controller.cpp
```cpp
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rrbot_controller::RRBotController, controller_interface::ControllerInterface)
```
#### Explicação do código
Esta macro é para registrar este controlador dentro do sistema de plug-ins do ROS2.

Crie um novo arquivo chamado `rrbot_controller.xml` no diretório raiz do pacote onde o arquivo CMakeLists.txt está localizado.

```bash
touch ~/ros2_ws/src/rrbot_controller/rrbot_controller.xml
```
Este é o conteúdo desse arquivo XML:
```xml
<library path="rrbot_controller">
  <class name="rrbot_controller/RRBotController"
         type="rrbot_controller::RRBotController" base_class_type="controller_interface::ControllerInterface">
  <description>
    RRBotController ros2_control controller.
  </description>
  </class>
</library>
```
Salve o arquivo antes de prosseguir para a próxima etapa.

Arquivo explicado
O elemento `< library>` especifica o caminho relativo para a biblioteca que contém o plug-in que você deseja exportar. Nesse caso, é `rrbot_controller`.

A tag `< class...>` declara o plug-in que você deseja exportar. Vamos aos seus atributos:

* `name`: o nome do plug-in ROS2_control.
* `type`: o namespace e o nome da classe que implementa o plugin.
* `base_class_type`: o namespace e o nome da classe base da qual este plugin herda.

A tag de descrição aninhada contém uma descrição do plug-in e o que ele faz.

Nota: O arquivo xml do plugin deve declarar todos os plugins contidos em um pacote, neste caso é apenas um.

## Prepare os arquivos `CMakeLists.txt` e `package.xml`

Adicionar diretivas de compilação ao arquivo `CMakeLists.txt`
Esta etapa adiciona as diretivas de compilação necessárias em `CMakeLists.txt`, que você precisa para compilar seu pacote.

O arquivo `CMakeLists.txt` final deve ficar assim:

> CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.5)
project(rrbot_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(example_interfaces REQUIRED)

add_library(
  rrbot_controller
  SHARED
  src/rrbot_controller.cpp
)
target_include_directories(
  rrbot_controller
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  rrbot_controller
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)
# prevent pluginlib from using boost
target_compile_definitions(rrbot_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  controller_interface rrbot_controller.xml)

install(
  TARGETS
  rrbot_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  rrbot_controller
)
ament_export_dependencies(
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

ament_package()
```
#### Arquivo explicado
```cmake
add_library(
  rrbot_controller
  SHARED
  src/rrbot_controller.cpp
)

target_include_directories(
  rrbot_controller
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  rrbot_controller
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)
```
Aqui, adicione os arquivos de origem ao comando CMake `add_library()`. Observe também o uso de expressões geradoras com a sintaxe `$<.. : ..>` como argumentos para a diretiva `target_include_directories`.
```cmake
target_compile_definitions(rrbot_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
```
Esta linha impede que pluginlib use boost.
```cmake
pluginlib_export_plugin_description_file(controller_interface rrbot_controller.xml)
```
O comando CMake acima instalará o arquivo de descrição do plug-in (rrbot_controller.xml) para que o pluginlib possa carregar o plug-in. Os argumentos para este comando são:

* O pacote para a classe base, ou seja, `controller_interface`
* O caminho relativo para o xml de declaração do plug-in, neste caso apenas o nome do arquivo: `rrbot_controller.xml`

```cmake
install(
  TARGETS
  rrbot_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)
```
Isso copiará os binários gerados para `lib` e `include`. Também informamos ao CMake para instalar todos os arquivos de inicialização na pasta de inicialização.

```cmake 
ament_export_include_directories(
  include
)
ament_export_libraries(
  rrbot_controller
)
ament_export_dependencies(
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)
```
As linhas acima exportam o diretório include, a biblioteca e as dependências para uso por outros projetos.

## Adicione dependências ao arquivo package.xml
Normalmente, você precisa adicionar as dependências externas ao arquivo `package.xml` para que o ament possa construir o pacote.

Deixe este arquivo inalterado, pois neste exemplo de controlador personalizado não estamos usando nenhuma biblioteca externa.
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rrbot_controller</name>
  <version>0.0.0</version>

  <description>Controller for exemplary RRBot robot.</description>

  <maintainer email="bence.magyar.robotics@gmail.com">Bence Magyar</maintainer>
  <maintainer email="denis@stogl.de">Denis Štogl</maintainer>

  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>control_msgs</depend>
  <depend>controller_interface</depend>
  <depend>hardware_interface</depend>
  <depend>pluginlib</depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>realtime_tools</depend>
  <depend>example_interfaces</depend>

  <test_depend>ament_cmake_gmock</test_depend>
  <test_depend>controller_manager</test_depend>
  <test_depend>hardware_interface</test_depend>
  <test_depend>ros2_control_test_assets</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
Agora você deve ser capaz de compilar e corrigir quaisquer reclamações que possam aparecer.
```bash
cd ~/ros2_ws
colcon build --packages-select rrbot_controller
```
Funcionou? Parabéns, você está um passo mais perto de escrever seu próprio controlador! E se não funcionou desde o início, tente entender e resolver as mensagens de erro do seu compilador até compilar corretamente.

## Crie um arquivo de configuração para o gerenciador de controladores e controladores
Agora prossiga para adicionar os arquivos de configuração para ros2_control e o(s) controlador(es) que serão usados.

Seu arquivo de configuração estará dentro da pasta config, então adicione essa pasta ao seu pacote.

```bash
mkdir ~/ros2_ws/src/rrbot_controller/config
touch ~/ros2_ws/src/rrbot_controller/config/rrbot_controllers_custom.yaml
```
Com a pasta config instalada, copie as linhas de código mostradas abaixo e cole-as em um novo arquivo chamado `rrbot_controllers_custom.yaml`:
```yaml
# Controller manager configuration
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz

    # Define a name for controllers that we plan to use
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    rrbot_controller:
      type: rrbot_controller/RRBotController

# Properties of the custom controler and definition of joints to use
rrbot_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
    interface_name: position
```

Você deve atualizar o script `CMakeLists.txt` para instalar o novo arquivo de configuração no diretório de compartilhamento.
Para isso, de preferência, adicione o trecho abaixo após o comando install que instala os arquivos de cabeçalho em include:
```cmake
install(
  DIRECTORY
    config
  DESTINATION
    share/${PROJECT_NAME}/
)
```
## Atualize os parâmetros de configuração do plugin Gazebo
Agora falta apenas um pequeno detalhe: precisamos carregar nosso novo arquivo de configuração YAML. Da unidade 2 sabemos que ao trabalhar com Gazebo devemos referenciar este arquivo dentro dos parâmetros de configuração do plugin Gazebo. Vamos fazer isso agora.

Agora mude para o editor de código e abra o arquivo XACRO do robô `rrbot.xacro`.

Você pode encontrar o arquivo que precisa abrir aqui: `~/ros2_ws/src/ros2_control_course/unit2/rrbot_unit2/urdf`.

Localize as tags do plug-in Gazebo ros2_control e modifique o elemento correto para usar o novo arquivo de configuração `rrbot_controllers_custom.yaml`, em vez do que está definido atualmente.

É assim que a nova configuração do plug-in do Gazebo ros2_control deve ser:
```xml
  <!-- Gazebo's ros2_control plugin -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <robot_sim_type>gazebo_ros2_control/GazeboSystem</robot_sim_type>
      <parameters>$(find rrbot_controller)/config/rrbot_controllers_custom.yaml</parameters>
    </plugin>
  </gazebo>
```
## Crie um novo arquivo de inicialização para gerar o robô e executar o novo controlador
A criação de um novo arquivo de inicialização é opcional, pois é possível iniciar e parar os controladores usando apenas a interface de linha de comando do gerenciador do controlador. Para completar, aqui está um exercício que explica como criar um novo arquivo de inicialização que gera o robô no Gazebo inicia o novo controlador.
Vamos aderir aos princípios de separação de interesses e colocar este arquivo de inicialização dentro do pacote `my_robot_bringup`, onde estamos mantendo os outros arquivos de inicialização.
```bash
touch ~/ros2_ws/src/my_robot_bringup/launch/rrbot_with_rrbot_controller.launch.py
```
É assim que o arquivo de inicialização deve se parecer:

>  rrbot_with_rrbot_controller.launch.py
```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction, LogInfo
from launch.event_handlers import OnProcessExit, OnExecutionComplete
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():

    rrbot_description_path = os.path.join(
        get_package_share_directory('rrbot_unit2'))

    xacro_file = os.path.join(rrbot_description_path,
                              'urdf',
                              'rrbot.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'robot'],
                        output='screen')

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rrbot_controller", "-c", "/controller_manager"],
    )


    return LaunchDescription([
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=spawn_entity,
                on_completion=[
                    LogInfo(
                        msg='Spawn finished, waiting 10 seconds to start controllers.'),
                    TimerAction(
                        period=10.0,
                        actions=[joint_state_broadcaster_spawner],
                    )
                ]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        ),
        spawn_entity,
        node_robot_state_publisher,
    ])
```
#### Arquivo explicado
```python
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rrbot_controller", "-c", "/controller_manager"],
    )
```
A diferença agora é que esse arquivo de inicialização iniciará um controlador com o nome rrbot_controller.

## Recompilando e testando
Agora você deve recompilar e fonte antes de seguir em frente e testar seu controlador no robô.

```bash
cd ~/ros2_ws
colcon build --packages-select rrbot_controller my_robot_bringup rrbot_unit2
```
Se você seguiu passo a passo até aqui, verá seu código compilar com sucesso.
```bash
source install/setup.bash
```