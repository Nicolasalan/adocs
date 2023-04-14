# Modelo de Implementação de Interface de Hardware

Esta unidade descreve como escrever uma interface de hardware personalizada mínima para conectar um robô ao framework `ros2_control`. Esta unidade mantém as coisas o mais simples possível, mas explica o suficiente para demonstrar as várias etapas desse processo.

## Interfaces de hardware explicadas
Depois que o controlador faz seu trabalho, sua saída é enviada para uma interface de hardware. Uma interface de hardware é um componente de software que pode se comunicar com o `ROS2_control` em uma extremidade e um ou mais dispositivos de hardware na outra extremidade.

Por exemplo, você tem um sensor de posição que envia leituras de dados de posição. Essas leituras brutas são recebidas pela interface de hardware e encaminhadas para um controlador que escuta esses dados. Depois que o controlador faz seu trabalho, ele emite um novo comando de destino recebido pela interface de hardware. Em seguida, é transformado no formato de dados específico necessário e compreendido pelo dispositivo de hardware. Portanto, você pode pensar em uma interface de hardware como um intermediário e interpretador que traduz os dados do hardware para o controlador e vice-versa.

Na realidade, a interface de hardware e o `ros2_control` são acoplados por uma interface de plug-in. Em termos técnicos, dizemos que o `ROS2_control` implementa uma "classe abstrata" e a interface de hardware implementa a "classe derivada". Dessa forma, o algoritmo de controle não precisa conhecer as especificidades do hardware. E como uma interface de hardware é um plug-in, ela não cria um nó ROS2.

## Sua interface de hardware em cinco etapas
Para implementar uma interface de hardware para um dispositivo de hardware, você precisará fazer o seguinte:

* Crie um pacote para sua interface de hardware
* Escreva um arquivo de cabeçalho
* Adicionar um arquivo de origem .cpp
* Prepare os arquivos CMakeLists.txt e package.xml para compilação
* Registrar um plug-in

::: details
IMPORTANTE: Uma interface de hardware é sempre feita sob medida para um dispositivo de hardware específico. Portanto, para se comunicar com um dispositivo de hardware, você deve incluir os drivers ou bibliotecas específicas desenvolvidas para esse dispositivo. Essas bibliotecas geralmente estão disponíveis no site do vendedor ou do fabricante. Nesta unidade, você não integrará nenhuma biblioteca de hardware/comunicação. Em vez disso, forneceremos um modelo que você poderá adaptar posteriormente para incluir uma biblioteca de hardware para se comunicar com um dispositivo específico de sua escolha. Seremos mais específicos na próxima unidade e criaremos uma interface de hardware personalizada para servos Dynamixel, mas ainda seguiremos os mesmos passos apresentados aqui.
:::

## Crie um pacote para sua interface específica de hardware
Uma boa prática do ROS é colocar a interface de hardware em um pacote separado. Isso simplifica a dependência do pacote e torna seu código mais modular. Então siga os passos abaixo para construir sua primeira interface de hardware `ros2_control`!
Primeiro, vá para o diretório src dentro de ros2_ws:
```bash
cd ~/ros2_ws/src
```
Agora crie o novo pacote. Este novo pacote deve ter ament_cmake como um tipo de compilação e, como prática recomendada, sugerimos terminar o nome do novo pacote com "hardware_interface". Você também precisará de algumas dependências:

* hardware_interface
* pluginlib
* rclcpp
* rclcpp_lifecycle

```bash
ros2 pkg create --build-type=ament_cmake my_robot_hardware_interface --dependencies hardware_interface pluginlib rclcpp rclcpp_lifecycle
```
Seu diretório de pacotes agora deve ter um CMakeLists.txt e um arquivo package.xml e um src e um diretório `include/my_robot_hardware_interface`.

###  Add a header file (.hpp)
Agora, dentro da pasta `include/my_robot_hardware_interface`, crie um arquivo chamado `my_robot_hardware_interface.hpp`. Em seguida, adicione o seguinte trecho de código nesse arquivo:
> my_robot_hardware_interface.hpp
```cpp
#ifndef ROS2_CONTROL_DEMO_HARDWARE__RRBOT_SYSTEM_HPP_
#define ROS2_CONTROL_DEMO_HARDWARE__RRBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ros2_control_demo_hardware
{
class RRBotSystemPositionOnlyHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RRBotSystemPositionOnlyHardware);

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read() override;

  hardware_interface::return_type write() override;

private:
  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
};

}  // namespace ros2_control_demo_hardware

#endif 
```
#### O código explicado
Vamos detalhar o código do arquivo de cabeçalho acima e entendê-lo.
```cpp
#ifndef ROS2_CONTROL_DEMO_HARDWARE__RRBOT_SYSTEM_HPP_
#define ROS2_CONTROL_DEMO_HARDWARE__RRBOT_SYSTEM_HPP_
```
Aqui, adicione protetores de cabeçalho ao arquivo de cabeçalho para garantir que ele não seja incluído mais de uma vez.
```cpp
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
```
Aqui, use as diretivas `#include` para incluir o conteúdo dos arquivos especificados no compilador.
```cpp
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
```
Isso permite que você use `CallbackReturn::SUCCESS` e `CallbackReturn::ERROR` em seu código.
```cpp
namespace ros2_control_demo_hardware
```
Aqui, coloque sua implementação em um namespace que corresponda ao nome do pacote.
```cpp
class RRBotSystemPositionOnlyHardware : public hardware_interface::SystemInterface
```
Em seguida, defina a classe com um nome para a interface e herde-a de hardware_interface::SystemInterface. Ao fazer isso, você está dizendo a `ros2_control` que esta é uma interface de hardware para um sistema.

Também é possível herdar de `hardware_interface::ActuatorInterface` ou `hardware_interface::SensorInterface`. No entanto, não iremos implementá-los porque um sistema inclui a funcionalidade de um sensor e atuador, portanto, não há muita diferença.
```cpp
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RRBotSystemPositionOnlyHardware);
```
Adicione esta linha como um membro público para utilizar esta classe com ponteiros compartilhados.
```cpp
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read() override;

  hardware_interface::return_type write() override;
```
Em seguida, declare alguns métodos que sua interface de hardware pode implementar ou substituir.

Explicaremos cada um desses métodos em um momento no arquivo fonte `.cpp`.
```cpp
private:
  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
};

}  // namespace ros2_control_demo_hardware

#endif 
```
Frequentemente, os drivers de hardware exigem que determinados parâmetros sejam definidos. Nesse caso, você pode adicionar variáveis protegidas à classe para armazenar esses valores.

###  Add a .cpp source file

Agora adicione o arquivo fonte correspondente para o cabeçalho dentro da pasta src do pacote.

Para continuar, crie um arquivo chamado `my_robot_hardware_interface.cpp` que conterá o seguinte código:
```cpp
#include "my_robot_hardware_interface/my_robot_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_hardware
{
    
} // namespace ros2_control_demo_hardware 
```
Salve seu trabalho antes de fazer qualquer outra alteração.
#### O código explicado

```cpp
#include "my_robot_hardware_interface/my_robot_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
```
Primeiro, inclua os arquivos de cabeçalho necessários como de costume.

```cpp

namespace ros2_control_demo_hardware
{
    
} // namespace ros2_control_demo_hardware 
```
Adicione uma definição de namespace para simplificar o desenvolvimento adicional. Por exemplo, quase todo o código que você adicionará abaixo deve ser colocado dentro desses colchetes de namespace.
### Escreva o método on_init()
O método on_init() é o primeiro método chamado. Use-o para inicializar todas as entidades de dados necessárias para executar a interface de hardware e, opcionalmente, validar os parâmetros lidos do arquivo URDF.

> my_robot_hardware.cpp
```cpp
CallbackReturn RRBotSystemPositionOnlyHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // START: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}
```
#### O código explicado
```bash
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
```
Na primeira linha, chame o método `on_init()` da classe pai para analisar e buscar dados da descrição `URDF/XACRO` do robô.
```cpp

  // START: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
```
Aqui você está inicializando variáveis de membros privados para os parâmetros que serão definidos de dentro do arquivo URDF/XACRO.
```cpp
 for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' has %zu command inCallbackReturn RRBotModularJoint::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
terfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
  }
```
Posteriormente, este exemplo mostra quatro verificações cruzadas opcionais usadas para validar antecipadamente se todas as juntas e interfaces no arquivo URDF têm valores permitidos ou uma combinação de valores. A primeira verificação é verificar se o número de interfaces de comando analisadas do arquivo URDF é diferente de 1. Em seguida, ele valida se a interface de comando é um tipo de "interface de posição". Depois, se o número de interfaces de estado também for igual a 1. Por fim, verifica se a interface de estado utilizada está de acordo com o tipo "interface de posição". Se alguma dessas condições não for atendida, o método de inicialização retornará um erro.
```cpp
return CallbackReturn::SUCCESS;
```
Se todos os parâmetros necessários forem definidos e válidos e tudo funcionar bem, você retornará `CallbackReturn::SUCCESS`.

###  Add the on_configure() method
O método `on_configure()` é usado para iniciar a comunicação com o hardware e para garantir que os estados do hardware possam ser lidos.

Aqui está um trecho de código C++ que implementa um modelo de código para `on_configure()`, adicione-o ao seu arquivo de código-fonte.

```cpp
CallbackReturn RRBotSystemPositionOnlyHardware::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  // START: This part here is for exemplary purposes - Please do not copy to
  // your production code

  // prevent unused variable warning
  auto prev_state = previous_state;
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
              "Configuring ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
                "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your
  // production code

  // reset values always when configuring hardware
  for (uint i = 0; i < hw_states_.size(); i++) {
    hw_states_[i] = 0;
    hw_commands_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
              "Successfully configured!");

  return CallbackReturn::SUCCESS;
}
```
Este modelo exemplar é bastante simples, como você pode ver, ele cria algumas mensagens de log confirmando que o hardware entrou na fase de configuração e, em seguida, redefine o estado e os valores de comando na memória.

### Adicione o método export_state_interfaces()

O método `export_state_interfaces()` é usado para definir as interfaces que seu hardware oferece.

Adicione o seguinte bloco de código em seu arquivo `my_robot_hardware.cpp` logo abaixo do método anterior, mas ainda dentro dos colchetes de namespace.

```cpp
std::vector<hardware_interface::StateInterface>
RRBotSystemPositionOnlyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}
```
#### O código explicado

```cpp
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }
```
Esse código percorre todas as juntas recuperadas do arquivo de configuração e define `hardware_interface::HW_IF_POSITION` como o único estado disponível no dispositivo de hardware.

Outros estados possíveis são `HW_IF_VELOCITY` para velocidade, `HW_IF_ACCELERATION` para aceleração e `HW_IF_EFFORT` para esforço. Por fim, você deve exportar o tipo de valor de interface de hardware que seu hardware fornece.

### Crie o método export_command_interfaces()
Que tipo de comandos de entrada seu hardware entende?

Sua API de firmware pode ter diferentes modos de operação. Por exemplo, modos de controle de posição, velocidade e corrente podem estar disponíveis. 
::: details
Novamente, você terá que consultar a documentação do seu atuador específico para descobrir quais modos de operação são suportados pelo seu hardware.
:::

Como o `ROS2_control` não sabe automaticamente quais modos de controle seu driver de hardware suporta, você precisa registrá-los manualmente usando este método.

O código a ser adicionado ao seu arquivo `my_robot_hardware.cpp` é o seguinte.
```cpp
std::vector<hardware_interface::CommandInterface>
RRBotSystemPositionOnlyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}
```
#### O código explicado

```cpp
 std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
```
Aqui, você cria um novo vetor command_interfaces e depois passa quais tipos de comandos são aceitos pelo hardware. Neste exemplo, você informa ao ROS2_control que seu hardware aceita apenas comandos de posição (HW_IF_POSITION).

### Implemente o método on_activate()
Este método executa a sequência de comandos que irão ligar seu hardware para permitir o movimento.

Prossiga para adicionar o método `on_activate()` mostrado abaixo no final do método `export_command_interfaces()` que você adicionou anteriormente.

```cpp
CallbackReturn RRBotSystemPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // START: This part here is for exemplary purposes - Please do not copy it to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Successfully activated!");

  return CallbackReturn::SUCCESS;
}
```
#### O código explicado
Como você não pode se comunicar com o firmware do atuador real, exiba mensagens de log na tela do console, lembre-se de que este é um modelo e que você deve incluir seu atuador ou biblioteca específica do robô e métodos para fazer interface com seu hardware específico de dentro deste `on_activate()` método.

### Implemente o método read()
O método `read()` é obter os estados do hardware e armazená-los em variáveis internas definidas em `
export_state_interfaces()`. O loop do controlador principal então usa esses valores para fazer seu trabalho.

Aqui está um esboço de código C++ que implementa um método `read()` simulado:
```cpp
hardware_interface::return_type RRBotSystemPositionOnlyHardware::read()
{
  // START: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Reading...");

  for (uint i = 0; i < hw_states_.size(); i++)
  {
    // Simulate RRBot's movement
    hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]) / hw_slowdown_;
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Got state %.5f for joint %d!",
      hw_states_[i], i);
  }
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Joints successfully read!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}
```
O código explicado
Como você não pode ler os dados provenientes do firmware do atuador real, exiba mensagens de log na tela do console e lembre-se de que este é um modelo e que você deve incluir seu atuador ou biblioteca específica do robô e métodos para fazer interface com seu hardware específico de dentro deste método `read()`.

### Implemente o método write()
A função `write()` é chamada no loop de controle após o cálculo do comando de controle para produzir sinais de atuação para o hardware.

Para implementar o método `write()`, cole o código a seguir no final do arquivo que você ainda deve ter aberto na seção anterior.
```cpp
hardware_interface::return_type RRBotSystemPositionOnlyHardware::write()
{
  // START: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Writing...");

  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Got command %.5f for joint %d!",
      hw_commands_[i], i);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}
```
#### O código explicado
```cpp
  // START: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Writing...");

  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Got command %.5f for joint %d!",
      hw_commands_[i], i);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code
```
Como você não pode emitir um comando para nenhum firmware de atuador real, use instruções de impressão para exibir mensagens na tela do console. Novamente, lembre-se de incluir seu atuador ou biblioteca específica do robô e métodos para fazer interface com seu hardware específico de dentro deste método `write()` mostrado aqui.

### Adicione a macro PLUGINLIB_EXPORT_CLASS

Depois que o namespace for fechado, adicione uma chamada à macro `PLUGINLIB_EXPORT_CLASS` no final do arquivo `.cpp`.

Adicione o seguinte bloco de código na parte inferior do seu arquivo de código-fonte:
```cpp
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_hardware::RRBotSystemPositionOnlyHardware, hardware_interface::SystemInterface)
```

Explicação do código
Isso é para registrar a classe dentro do sistema de plugins.

### Escrevendo Definição de Exportação para `pluginlib`
Com o código do seu plug-in pronto, o pacote deve compilar, mas você ainda não conseguiria carregar o plug-in dinamicamente porque ros2_control não saberia de sua existência. Você precisa adicionar um arquivo `.xml` para listar o plugin definido e depois exportá-lo.

Crie um novo arquivo chamado `my_robot_hardware_interface.xml` no diretório raiz do pacote onde o arquivo CMakeLists.txt também está definido.
```bash
cd ~/ros2_ws/src/my_robot_hardware_interface
touch my_robot_hardware_interface.xml
```
Este é o conteúdo desse arquivo XML:
```xml

<library path="my_robot_hardware_interface">
  <class name="ros2_control_demo_hardware/RRBotSystemPositionOnlyHardware"
         type="ros2_control_demo_hardware::RRBotSystemPositionOnlyHardware"
         base_class_type="hardware_interface::SystemInterface">
    <description>
      ros2_control minimal hardware interface template
    </description>
  </class>
</library>
```
Salve o arquivo antes de prosseguir para a próxima etapa.

#### Arquivo explicado
O elemento `< library>` especifica o caminho relativo para a biblioteca que contém o plug-in que você deseja exportar. Neste caso, é ros2_control_demo_hardware.

A tag `< class...`> declara o plug-in que você deseja exportar. Vamos aos seus atributos:

* name: o nome do plug-in ROS2_control que você definiu no arquivo URDF usando a tag `< plugin>`.
* type: o namespace e o nome da classe que implementa o plugin.
* base_class_type: o namespace e o nome da classe base da qual este plugin herda.

A tag de descrição aninhada contém uma descrição do plug-in e o que ele faz.

### Prepare os arquivos CMakeLists.txt e package.xml

#### Adicionar diretivas de compilação ao arquivo CMakeLists.txt
Esta etapa adiciona as diretivas de compilação necessárias em `CMakeLists.txt`, que você precisa para compilar seu pacote.

O arquivo `CMakeLists.txt` final deve ficar assim:
```cmake
cmake_minimum_required(VERSION 3.5)
project(my_robot_hardware_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)


## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/my_robot_hardware_interface.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface my_robot_hardware_interface.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()
```
#### Arquivo explicado
```cmake
## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/my_robot_hardware_interface.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
```
Aqui, adicione os arquivos de origem ao comando CMake `add_library()`.
```cmake
pluginlib_export_plugin_description_file(hardware_interface my_robot_hardware_interface.xml)
```
Adicione esta linha para colcon build para instalar um arquivo com um caminho para o arquivo `.xml` de descrição do plug-in no índice de recursos. Com esta informação, o pluginlib pode carregar sua biblioteca de interface de hardware.
```cmake
# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}/
)
```

Isso copiará os binários gerados para lib e incluirá. Também informamos ao CMake para instalar todos os arquivos de inicialização na pasta de inicialização.
```cmake

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()
```
Exporte os executáveis ou bibliotecas para uso por outros projetos.
#### Adicione dependências ao arquivo package.xml
Normalmente, você precisa adicionar as dependências externas ao arquivo `package.xml` para que o ament possa construir o pacote.

Deixe este arquivo inalterado, pois você não está usando nenhuma biblioteca externa neste modelo de interface de hardware exemplar.
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_hardware_interface</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>hardware_interface</depend>
  <depend>pluginlib</depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
### Alternando o plug-in de hardware
O plug-in de interface de hardware do Gazebo definido dentro do arquivo URDF ou XACRO deve ser substituído por um elemento de plug-in XML diferente para que possamos trabalhar com hardware real? Faça isso agora.

Use a janela do editor de código para ir para a pasta `/ros2_ws/src/ros2_control_course/unit3` e depois para `/rrbot_unit3/urdf`

Modifique o arquivo rrbot.xacro para usar seu plug-in de interface de hardware adicionando o seguinte código no final do arquivo antes do elemento `< /robot>` de fechamento.

```xml
  <ros2_control name="DemoHardware" type="system">
  
      <hardware>   
        <plugin>ros2_control_demo_hardware/RRBotSystemPositionOnlyHardware</plugin>
        <param name="example_param_hw_start_duration_sec">0.0</param>
        <param name="example_param_hw_stop_duration_sec">3.0</param>
        <param name="example_param_hw_slowdown">10.0</param>
      </hardware>
      
      <joint name="joint1">
        <command_interface name="position">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>
        <state_interface name="position"/>
      </joint>
      <joint name="joint2">
        <command_interface name="position">
          <param name="min">-1</param>
          <param name="max">1</param>
        </command_interface>
        <state_interface name="position"/>
      </joint>

    </ros2_control>
```
Salve seu progresso antes de passar para a parte final desta unidade.

Observe as tags `< param name="..">< /param>`. ros2_control obtém os valores entre essas tags para serem acessados de dentro do código do plug-in da interface de hardware.

E o plug-in Gazebo?

Ao trabalhar com um robô real, a tag `< gazebo>` dentro de nosso arquivo URDF ou XACRO é ignorada. Você simplesmente não precisa disso quando conectado a um robô real. Mas, espere um segundo! Você precisa do plug-in `ros2_control` do Gazebo para carregar o arquivo de configuração do controlador `ros2_control` `.yaml`, não é? Sim absolutamente! Boa pegada! Você pode consultar a Unidade 2 para revisar como você definiu e passou no arquivo de configuração `.yaml`, se necessário.

Como você não está usando o Gazebo, você precisa encontrar uma maneira diferente de passar no arquivo de configuração do controlador. A solução para esse problema é gravar um novo arquivo de inicialização que você usará apenas ao executar em hardware real. Tudo bem então, vamos fazer isso!

#### Crie um novo arquivo de inicialização para hardware real
Comece criando um diretório para armazenar seus arquivos de inicialização e crie um novo arquivo chamado `bring_up_on_hardware.launch.py` dentro.
```bash
cd ~/ros2_ws/src/my_robot_hardware_interface
mkdir launch
cd launch
touch bring_up_on_hardware.launch.py
```
É assim que o arquivo de inicialização deve se parecer:
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    robot_name = "rrbot"
    robot_model_file = "rrbot.xacro"
    package_name = "rrbot_unit3"

    rviz_config = os.path.join(get_package_share_directory(
        "rrbot_unit3"), "rviz", "unit3.rviz")

    robot_description = os.path.join(get_package_share_directory(
        package_name), "urdf", robot_model_file)

    robot_description_config = xacro.process_file(robot_description)

    controller_config = os.path.join(
        get_package_share_directory(
            "my_robot_hardware_interface"), "config", "controller_configuration.yaml"
    )

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description_config.toxml()}, controller_config],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster",
                       "--controller-manager", "/controller_manager"],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["forward_position_controller",
                       "-c", "/controller_manager"],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller",
                       "-c", "/controller_manager"],
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_config.toxml()}],
            output="screen"),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output={
                "stdout": "screen",
                "stderr": "log",
            },
        )

    ])
```
#### Arquivo explicado
Vejamos as partes importantes do código aqui:
```python
    controller_config = os.path.join(
        get_package_share_directory(
            "my_robot_hardware_interface"), "config", "controller_configuration.yaml"
    )
# Code omitted for brevity
    
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description_config.toxml()}, controller_config],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
        ),
```
A diferença agora é que você precisa iniciar o executável `ros2_control_node` separadamente e fornecer o arquivo de configuração `.yaml` com a configuração do gerenciador do controlador e dos controladores.

```python
    Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output={
                "stdout": "screen",
                "stderr": "log",
            },
```
Além disso, como você não está usando o Gazebo, agora está iniciando o RVIZ2 para visualizar o estado do robô que o `joint_state_broadcaster` transmite.

#### Crie um arquivo de configuração para o gerenciador de controladores e controladores

Agora prossiga para adicionar os arquivos de configuração para `ros2_control` e o(s) controlador(es) que serão usados.

Seu arquivo de configuração estará dentro da pasta config, então adicione essa pasta ao seu pacote.
```bash
cd ~/ros2_ws/src/my_robot_hardware_interface
mkdir config
```
Com a pasta config instalada, copie as linhas de código mostradas abaixo e cole-as em um novo arquivo chamado `controller_configuration.yaml`:

```yaml
# Controller manager configuration
controller_manager:
  ros__parameters:
    update_rate: 10  # Hz

    ### Controllers to be initialized at startup ###
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    forward_position_controller:
      type: forward_command_controller/ForwardCommandController

    position_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController


### Properties of the controllers that we will use and definition of joints to use ###
forward_position_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
    interface_name: position


position_trajectory_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2

    command_interfaces:
      - position

    state_interfaces:
      - position

    state_publish_rate: 200.0 # Defaults to 50
    action_monitor_rate: 20.0 # Defaults to 20

    allow_partial_joints_goal: false # Defaults to false
    open_loop_control: true
    allow_integration_in_goal_trajectories: true
    constraints:
      stopped_velocity_tolerance: 0.01 # Defaults to 0.01
      goal_time: 0.0 # Defaults to 0.0 (start immediately)
```
### Compilando e testando a interface de hardware
Agora tudo está pronto e você pode compilar seu pacote de interface de hardware.
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select my_robot_hardware_interface rrbot_unit3
```
Se a compilação foi bem-sucedida, obtenha o arquivo setup.bash da pasta de instalação.
```bash
source install/setup.bash
```
Você está pronto para executar seu arquivo de inicialização, o que você faz da seguinte forma:
```bash
ros2 launch my_robot_hardware_interface bring_up_on_hardware.launch.py
```

Estas são as mensagens de log que os métodos `read()` e `write()` imprimem na tela do console enquanto o algoritmo de controle é executado.

Após alguns segundos deve aparecer uma nova janela no topo da tela principal, esta é a janela da Interface Gráfica.

Tudo começou corretamente se você pode ver o robô em pé no RVIZ.
Execute a `ros2 topic list` no shell para ver os tópicos do ROS2 atualmente disponíveis:
```bash
ros2 topic list
```
Você deve ver este tópico na lista:
```bash
/forward_position_controller/commands
```
Verifique se os controladores estão ativos.
```bash
ros2 control list_controllers
```
Você deve ver esta saída do console:
```bash
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
forward_position_controller[forward_command_controller/ForwardCommandController] active
```
Publique uma sequência de comandos de movimento para ver como o robô pode mover suas juntas.
```bash
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
- 1.57
- 0.0" -1
```