# Princípios de Design de Árvores Comportamentais no Contexto de uma Tarefa Bem-sucedida Descrição do Agente Autônomo

## Arquitetura de Árvores Comportamentais
Na unidade anterior, introduzimos o conceito de BT e definimos primitivas lógicas para arquitetar o comportamento de agentes autônomos (um robô). Agora você entende a pilha de abstração e pode localizar BT nela.

Embora os BTs sejam razoavelmente simples de compreender e usar, é benéfico estar ciente dos princípios e métodos de design que podem ser usados em muitos contextos para realizar plenamente seu potencial. Vários exemplos serão usados para ilustrar essas ideias na unidade seguinte.

Especificamos a interpretação lógica de como o robô se comporta durante a execução da tarefa conforme refletido no arquivo XML.

Neste curso, você usa o `framework BehaviourTree.CPP`, que habilita e protege as tarefas do robô que serão executadas de acordo com as definições lógicas (escritas em XML). Isso significa que o robô percorre o BT de acordo.

A pilha a seguir ilustra as relações e ações a serem executadas usando o BT para modelar a tarefa do robô. O diagrama fornece uma visão geral do fluxo de dados durante a execução do aplicativo do robô incorporado ao BT.

<div align="center">
     <img src="./img/worrkflow.png" alt="Particles Filter" width="700px">
</div>

#### Estudar este diagrama é essencial para entender BT e sua relação com ROS.

Imagine que seu robô deve executar a tarefa (por exemplo, limpar o chão). A tarefa consiste em várias ações, muitas vezes relacionadas à interação com o ambiente do robô. A aplicação do robô geralmente inclui mecanismos de supervisão de segurança e outras "verificações" (verificação da bateria). As ações do robô e mecanismos auxiliares são modelados em BT, onde as relações lógicas são expostas no arquivo XML.

Usando a estrutura BehaviourTree.CPP, você cria a declaração de nós (seguindo o XML; arquitetura BT) e definições. Então, criando as definições dos nós, você expressa as ações executadas neste nó BT.

Um conjunto de ações definidas por um nó BT é considerado um callback (BT chama outras funções/ações). Neste curso, você define um conjunto de ações executadas no nó ROS. Além disso, o nó ROS se comunica com um robô ou simulação (no seu caso, Gazebo). Os nós ROS recebem o feedback e enviam o estado (feedback) para o nó BT. A análise de feedback afeta o nó BT e envia o retorno para a Raiz (SUCCESS, FAILURE ou RUNNING).

Tendo em mente a pilha de abstrações definida anteriormente, você pode avaliar novamente o BT, que define o fluxo lógico. O projeto detalhado dos nós ROS é desvinculado do conceito BT.

Você não considera em detalhes como o nó executa a tarefa (por exemplo, calcula a cinemática inversa). No entanto, você considera apenas o comportamento do nó, ou seja, a funcionalidade do nó (como uma “caixa de bloco”). Ao arquitetar o BT, você sabe o que o nó ROS está fazendo, mas o detalhamento da implementação não é obrigatório nesse raciocínio abstrato.

Nas próximas seções, discutiremos as vantagens de utilizar a estrutura de arquivo XML explícita e a abordagem para modelar a estrutura lógica de BT. Além disso, você experimentará o mapeamento do XML na estrutura BehaviorTree.CPP e ROS2.

Você abordará uma visão mais profunda da BT e dos mecanismos que orquestram as definições lógicas das tarefas do robô. Referindo-se às suas suposições anteriores, arquitetando o BT, considere apenas o nível mais alto de abstração, evitando uma compreensão mais profunda dos mecanismos relacionados à implementação do nó ROS. O nó, como mencionado, é considerado uma “caixa preta” – executando uma função no sistema de robô que você constrói.

Considere o diagrama abaixo descrevendo conexões lógicas em BT. Comece pela função primária, que lê o arquivo XML.

A função primária e a estrutura BT incluem a declaração de nós. A estrutura BehaviourTree.CPP gerencia o fluxo lógico (marque o nó BT) de acordo com uma especificação XML (invoca um retorno de chamada).

Os nós BT executam seu callback (conjunto de ações). O retorno de chamada retorna SUCCESS, FAILURE ou RUNNING. A ação de retorno de chamada pode se comunicar com os nós ROS. Os nós ROS executam ações relacionadas ao robô.

<div align="center">
     <img src="./img/root.png" alt="Particles Filter" width="700px">
</div>

## Tipos de nó e comportamento. O Modelo de Fluxo Lógico BT em Formato XML

Aborde os desafios com o design conceitual de BT, que por natureza deve ser legível por humanos, introduzindo o conceito XML. O BehaviorTree.CPP oferece uma linguagem de script XML que suporta humanos para especificar árvores específicas e nós únicos.

Para arquitetar o BT de cada tarefa do robô, você definiu brevemente os primitivos BT (nós) na unidade anterior. Os nós Sequence, FallBack e Decorators foram descritos. Isso ajudou você a entender o BT em geral. No entanto, você deve adicionar mecanismos e abstrações lógicas para ser mais consistente e aderir ao conceito de framework BehaviorTree.CPP.

Para fornecer uma compreensão completa da arquitetura BT, veja abaixo uma variedade de técnicas e abstrações de estrutura na seção a seguir. A palestra seguirá os princípios orientadores da abordagem de design XML. No entanto, você se concentrará principalmente em uma abordagem prática (usando exemplos C++).

Por favor, esteja familiarizado com a hierarquia de tipo de nó representada no diagrama abaixo.

<div align="center">
     <img src="./img/type.png" alt="Particles Filter" width="700px">
</div>

Primeiro, considere o nó Sequência. O último diagrama descreve três tipos de nós de sequência. Na tabela a seguir, você pode entender o comportamento de cada um deles.

<div align="center">
     <img src="./img/tab2.png" alt="Particles Filter" width="700px">
</div>

Na tabela, Restart e Tick novamente podem ser compreendidos da seguinte forma:

* Reiniciar significa que todo o nó de sequência é reiniciado a partir do primeiro filho na lista. Se o nó de sequência incluir os filhos A, B, C e D, então A e B são SUCCESS. No entanto, C é FALHA. O próximo tick obriga você a verificar novamente, começando de A.

* Marcar novamente significa que na próxima vez que a sequência for marcada, o mesmo filho será marcado. Os filhos anteriores lembram-se do status. Se o nó de sequência incluir os filhos A, B, C e D, então A e B são SUCCESS. No entanto, C é FALHA. O próximo tick força você a verificar novamente C, NOT A e B, quais estados são lembrados.

Revise os exemplos simples, as definições lógicas em XML e os exemplos em C++. A estrutura BehaviorTree.CPP será estudada cuidadosamente na próxima unidade.

<div align="center">
     <img src="./img/seq2.png" alt="Particles Filter" width="700px">
</div>

A sequência a seguir é simples.

* O robô se move em linha reta e para frente, lendo o sensor laser. As leituras são precisas e fornecem uma excelente visão geral dos arredores. T
* O robô analisa os dados do sensor e percebe as condições para evitar obstáculos que possam aparecer nas proximidades do robô (enquanto se move em linha reta). Se o obstáculo estiver a menos de dois metros do robô `(CONDIÇÃO 1)`, o robô deve parar.
* O robô deve novamente analisar o ambiente e decidir como evitar o obstáculo (neste caso, assuma que o robô pode virar à esquerda ou à direita; o robô escolhe a direção aleatoriamente). O robô verifica se há energia suficiente armazenada na bateria para girar `(CONDIÇÃO 2)` antes do robô girar (esquerda ou direita). Nesse caso, o robô gira 90 graus `(AÇÃO 1)` e se move em linha reta `(AÇÃO 2)`.

Agora, você pode arquitetar o BT para essas ações do robô.

```xml
<root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <Obstacle   name=" Obstacle detected"/>
            <EnoughBattery   name=" Battery OK"/>
            <Rotate       name=" Rotated"/>
            <MoveStraight name=" Moved"/>
        </Sequence>
    </BehaviorTree>

</root>
```

Verifique a tabela descrita acima. Se a CONDIÇÃO 1 for FALHA (sem obstáculo), a sequência terminará e o nó Sequência será reiniciado.

Considere o caso em que o robô precisa de cinco segundos para girar antes que o callback retorne RUNNING. Nesse caso, a ação, rotacionar, obterá o tick subsequente e determinará se o estado mudou de RUNNING para SUCCESS. O nó de sequência lembra as verificações anteriores (CONDIÇÃO 1 e CONDIÇÃO 2) e não há necessidade de verificá-las novamente.

Além disso, você descreveu exemplos na unidade anterior e agora é uma ótima oportunidade para entender o design XML. Conforme discutido, XML é usado para modelar o fluxo lógico em BT. Neste exemplo, você pode arquitetar o nó de sequência da seguinte maneira.

Novamente, é crucial enfatizar que você projeta as conexões lógicas e o fluxo das ações aqui. Cada ação descrita nesta unidade possui um callback (do ponto de vista do site Raiz), o que implica que possui rotinas específicas a serem executadas.

Para simplificar, se o robô verificar a bateria, ele receberá os resultados SUCESSO (energia suficiente) ou FALHA do retorno de chamada (a bateria está vazia). Em um sistema real, verificar a bateria é significativamente mais difícil e envolve mais etapas do que apenas verificar.

> bt1.cpp
```cpp
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class Obstacle : public BT::SyncActionNode
{
  public:
    Obstacle(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        std::cout << "OK. Detected !"
                  << "\n";
        return BT::NodeStatus::SUCCESS;
    }
};

class EnoughBattery : public BT::SyncActionNode
{
  public:
    EnoughBattery(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        std::cout << "is there energy remain?: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

class Rotate : public BT::SyncActionNode
{
  public:
    Rotate(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        std::cout << "rotating 90 deg : " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

class MoveStraight : public BT::SyncActionNode
{
  public:
    MoveStraight(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        std::cout << "moving again straight : " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

static const char* xml_text = R"(

 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <Obstacle   name=" Obstacle detected"/>
            <EnoughBattery   name=" Battery OK"/>
            <Rotate       name=" Rotated"/>
            <MoveStraight name=" Moved"/>
        </Sequence>
     </BehaviorTree>

 </root>
 )";

int main()
{
    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

    // The recommended way to create a Node is through inheritance.
    // Even if it requires more boilerplate, it allows you to use more functionalities
    // like ports (we will discuss this in future tutorials).

    factory.registerNodeType<Obstacle>("Obstacle");
    factory.registerNodeType<EnoughBattery>("EnoughBattery");
    factory.registerNodeType<Rotate>("Rotate");
    factory.registerNodeType<MoveStraight>("MoveStraight");

    // Trees are created at deployment-time (i.e. at run-time, but only once at the beginning).
    // The currently supported format is XML.
    // IMPORTANT: when the object "tree" goes out of scope, all the TreeNodes are destroyed
    auto tree = factory.createTreeFromText(xml_text);

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    tree.tickRoot();

    return 0;
}
```
Execute no Shell:

```bash
source ~/ros2_ws/install/setup.bash
./bt1
```

> Output 
```bash
OK. Detected !
is there energy remain?:  Battery OK
rotating 90 deg :  Rotated
moving again straight :  Moved
```

A unidade a seguir permite uma compreensão profunda da arquitetura e dos mecanismos de BT, permitindo arquitetar a conexão lógica do comportamento do robô.

Este exemplo é semelhante ao executado na unidade anterior.

1. O robô executa tarefas de sequência.
2. Primeiro, o robô verifica se existe algum obstáculo, o que, neste caso, é verdadeiro. Nó imprime OK. Detectou!
3. Em seguida, o robô verifica se tem bateria suficiente para realizar as tarefas de desvio. A verificação é SUCESSO. O nó imprime: há energia restante?: Bateria OK.
4. Em seguida, o robô gira. Impressões de nó: girando 90 graus: girado.
5. Finalmente, o robô se move em linha reta. Nó imprime: movendo-se novamente em linha reta: movido.

## Esquema XML
Observe que, apesar da biblioteca ser desenvolvida em C++, os BTs ainda podem ser criados durante o tempo de execução. O XML pode ser lido de um arquivo ou incorporado ao código C++.
```xml
 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <SaySomething   name="action_hello" message="Hello"/>
            <OpenGripper    name="open_gripper"/>
            <ApproachObject name="approach_object"/>
            <CloseGripper   name="close_gripper"/>
        </Sequence>
     </BehaviorTree>
 </root>
```
Seguindo o padrão descrito:

* A primeira marca de árvore é a Raiz. Deve conter uma ou mais tags, BehaviorTree.
* A tag BehaviorTree deve ter o atributo ID.
     * A tag Root deve conter o atributo main_tree_to_execute.
     * O atributo `main_tree_to_execute` é obrigatório se o arquivo contiver vários `BehaviorTrees`; opcional caso contrário.
* Uma única tag representa cada nó da árvore. Em particular:

     * O nome do tag é o ID usado para registrar o TreeNode na fábrica.
     * O nome do atributo refere-se ao nome da instância e é opcional.
     * As portas são configuradas usando atributos.
Quanto ao número de filhos:

* **ControlNodes** contém de 1 a N filhos.
* **DecoratorNodes** e subárvores contêm apenas 1 filho.
* **ActionNodes** e ConditionNodes não têm filhos.

### Nós aninhados

Os conceitos básicos de projeto devem ser mantidos em mente quando você arquiteta o BT. Por exemplo, o BT é realizado de cima para baixo e da esquerda para a direita. Usando esses conceitos, você pode construir a figura a seguir e deduzir os princípios de fluxo lógico XML.

Em seguida, examine o diagrama enquanto considera o relacionamento XML entre as operações.

<div align="center">
     <img src="./img/xml.png" alt="Particles Filter" width="600px">
</div>

```xml
 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
                <Sequence name="sequence_1">
                        <Sequence name="sequence_1">
                            <Condition1   name="condition_1"/>
                            <Action1      name="action_1"/>
                        </Sequence>
                        <Fallback name="fallback_1">
                            <Action2      name="action_2"/>
                            <Action3      name="action_3"/>
                        </Fallback>
                </Sequence>
                <Fallback name="fallback_2">
                         <Sequence name="sequence_3">
                            <Condition2   name="condition_2"/>
                            <Action4      name="action_4"/>
                        </Sequence>
                        <Sequence name="sequence_4">
                            <Condition3   name="condition_3"/>
                            <Action5      name="action_5"/>
                        </Sequence>
                </Fallback>
        </Sequence>
     </BehaviorTree>

 </root>
```
### Nós assíncronos (nó sequencial reativo)
Como afirmado anteriormente, o Nó Sequencial Reativo é uma melhoria para o Nó Sequencial, particularmente quando o Nó Condicional é permanentemente verificado antes de retornar SUCCESS. Mas, novamente, revise a referência da tabela, que mostra que toda a sequência foi redefinida.

Este tipo de sequência (Sequencial Reativo) é empregado quando a ação do nó é assíncrona. Isso denota que, até terminar, o nó retorna RUNNING (a ação executada pelo nó leva mais tempo do que o tempo de amostra do tick). Pense na ilustração resumida.

<div align="center">
     <img src="./img/rec.png" alt="Particles Filter" width="500px">
</div>

Por favor, considere o diagrama abaixo, que deve lhe dar uma melhor compreensão do que acontece durante a execução deste exemplo. Como não estamos discutindo a implementação de C++, recomendamos que você execute o programa e estude o código.

<div align="center">
     <img src="./img/time.png" alt="Particles Filter" width="500px">
</div>

Agora você explorará um aspecto crucial do BT e contrastará o nó sequencial descrito anteriormente com a sequência reativa no contexto do nó síncrono versus nó assíncrono.

Primeiro, observe que o thread para uma ação assíncrona é separado (próprio thread). Ao fazer isso, o usuário pode empregar métodos de bloqueio enquanto retorna o fluxo de execução para a árvore.

> bt2.cpp
```cpp
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class ObstacleCheck : public BT::SyncActionNode
{
  public:
    int count;

    ObstacleCheck(const std::string& name) : BT::SyncActionNode(name, {})
    {
        this->count = 0;
    }

    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        std::cout << "OK. Detected !"
                  << "\n";
        return BT::NodeStatus::SUCCESS;
    }
};

class takeDecision : public BT::AsyncActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    takeDecision(const std::string& name, const BT::NodeConfiguration& config)
      : AsyncActionNode(name, config)
    {
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        {
            _halt_requested.store(false);

            int count = 0;

            while ((!_halt_requested) && (count < 5))
            {
                std::cout << "thinking for :: " << count << "\n";
                std::this_thread::sleep_for(std::chrono::seconds(1));
                count++;
            }

            std::cout << "[ Decision process: FINISHED ]" << std::endl;

            return _halt_requested ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
        }
    }

    virtual void halt() override;

  private:
    std::atomic_bool _halt_requested;
};

void takeDecision::halt()
{
    _halt_requested.store(false);

    std::cout << __FUNCTION__ << " called!"
              << "\n";
}

static const char* xml_text_reactive = R"(

 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <ReactiveSequence name="root">
            <ObstacleCheck   name=" Obstacle detected"/>       
                <takeDecision   name="take Decision"/>
        </ReactiveSequence>
     </BehaviorTree>

 </root>
 )";

void Assert(bool condition)
{
    if (!condition)
        throw RuntimeError("this is not what I expected");
}

int main()
{
    using std::chrono::milliseconds;

    BehaviorTreeFactory factory;

    factory.registerNodeType<ObstacleCheck>("ObstacleCheck");
    factory.registerNodeType<takeDecision>("takeDecision");

    std::cout << "\n------------ BUILDING A NEW TREE ------------" << std::endl;

    auto tree = factory.createTreeFromText(xml_text_reactive);

    NodeStatus status;
    status = tree.tickRoot();

    std::cout << "\n--- 1st executeTick() ---" << std::endl;
    status = tree.tickRoot();

    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    std::cout << "\n--- 2nd executeTick() ---" << std::endl;
    status = tree.tickRoot();

    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    std::cout << "\n--- 3rd executeTick() ---" << std::endl;
    status = tree.tickRoot();

    return 0;
}
```
Execute no Shell:
```bash
source ~/ros2_ws/install/setup.bash
./bt2
```

> Output 
```bash

------------ BUILDING A NEW TREE ------------
OK. Detected !

--- 1st executeTick() ---
OK. Detected !
thinking for :: 0

--- 2nd executeTick() ---
OK. Detected !

--- 3rd executeTick() ---
OK. Detected !
thinking for :: 1
thinking for :: 2
thinking for :: 3
thinking for :: 4
```

Pense em como o software é executado. A cada segundo, o BT é assinalado. Como o robô está se aproximando do obstáculo, você pode assumir que a condição retorna SUCCESS.

Cada vez que o nó de obstáculo é marcado, ele imprime: OK. Detectou!

A ação Tomar Decisão é então verificada. A ação retorna RUNNING porque é assíncrona (o robô precisa de cinco segundos para pensar). O nó assíncrono opera em um encadeamento diferente (independentemente) quando o nó de sequência reativa é retomado; assim, o tempo passa enquanto o robô pensa. Cada vez que o nó é marcado, ele imprime: pensando por: (número de segundos).

Após o segundo tick da árvore, você pode observar a saída do programa. Apesar da condição de obstáculo ter sido verificada novamente, os tempos de reflexão aumentaram. Após o terceiro tick da árvore, o comportamento do nó ainda se aplica. O processo assíncrono ainda está em execução e termina (o status do nó muda de RUNNING para SUCCESS) depois que o contador se esgota.

### Simultaneidade