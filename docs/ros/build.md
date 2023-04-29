# Sistema de compilação Python ROS2
* Como estruturar e iniciar programas ROS2 (pacotes e arquivos de inicialização)
* Como criar programas ROS2 básicos (baseados em Python)
* Conceitos básicos do ROS2: nós, bibliotecas de cliente, etc.

## O que é um Pacote?
O ROS2 usa pacotes para organizar seus programas. Você pode pensar em um pacote como todos os arquivos que um programa ROS2 específico contém; todos os seus arquivos CPP, arquivos Python, arquivos de configuração, arquivos de compilação, arquivos de inicialização e arquivos de parâmetros. Além disso, organizar seus programas ROS2 em pacotes torna muito mais fácil compartilhá-los com outros desenvolvedores/usuários.

No ROS2, você pode criar dois tipos de pacotes: pacotes Python e pacotes CMake (C++). Porém, vamos nos concentrar nos primeiros. Pacotes Python conterão executáveis Python.

Todo pacote Python terá a seguinte estrutura de arquivos e pastas:

* `package.xml` - Arquivo contendo meta-informações sobre o pacote (mantenedor do pacote, dependências, etc.).
* `setup.py` - Arquivo contendo instruções de como compilar o pacote.
* `setup.cfg` - Arquivo que define onde os scripts serão instalados.

`/< package_name>` - Este diretório sempre terá o mesmo nome do seu pacote. Você colocará todos os seus scripts Python dentro desta pasta. Observe que ele já contém um arquivo `__init__.py` vazio.

Alguns pacotes podem conter pastas extras. Por exemplo, a pasta de inicialização contém os arquivos de inicialização do pacote (você lerá mais sobre isso mais tarde).

Para resumir, você deve se lembrar do seguinte:

* Todo programa ROS2 que você deseja executar é organizado em um pacote.
* Todo programa ROS2 que você criar deve ser organizado em um pacote.
* Os pacotes são o principal sistema de organização para programas ROS2.

## Criar um pacote
Quando você deseja criar pacotes, precisa trabalhar em um espaço de trabalho específico conhecido como espaço de trabalho do ROS2. O espaço de trabalho do ROS2 é o diretório em seu disco rígido onde residem os pacotes do ROS2 para serem usados pelo ROS2. Normalmente, o diretório do espaço de trabalho do ROS2 é chamado `ros2_ws`.

Vá para o `ros2_ws` no seu shell:

```bash
cd ~/ros2_ws/
pwd
```
Vá para o `ros2_ws` no seu shell:

Dentro deste espaço de trabalho, existe um diretório chamado src. Esta pasta contém todos os pacotes deste espaço de trabalho. Toda vez que você quiser criar um pacote, você deve estar neste diretório src de um espaço de trabalho.

Para criar seu próprio espaço de trabalho, basta criá-lo como uma pasta normal com uma pasta src dentro.

Exemplo de como você criaria o espaço de trabalho:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
```
Para esta seção, no entanto, não é necessário criá-lo, pois já o criamos para você.
Neste ponto, você está finalmente pronto para criar seu próprio pacote! Para fazer isso, digite o seguinte em seu shell:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_package --dependencies rclpy
```
Algo parecido com a mensagem abaixo aparecerá em seu terminal:
Dentro do seu diretório src, isso cria um novo pacote com arquivos nele. Você verificará isso mais tarde. Agora vamos ver como este comando é construído:
```bash
ros pkg create <package_name> --build-type ament_python my_package --dependencies <package_dependencies>
```
O `< package_name>` é o nome do pacote que você deseja criar e o `< package_dependencies>` são os nomes de outros pacotes ROS2 dos quais seu pacote depende.

Observe também que estamos especificando `ament_python` como o tipo de compilação. Isso indica que estamos criando um pacote Python.

É uma boa ideia construir seu pacote depois que ele foi criado. É a maneira mais rápida de determinar se as dependências que você listou podem ser resolvidas e verificar se não há erros nos dados inseridos.
```bash
cd ~/ros2_ws/
colcon build
```
Crie o hábito de obter o setup.bash da pasta de instalação para que o ROS possa encontrar os pacotes no espaço de trabalho.
```bash
source install/setup.bash
```
Os pacotes são organizados dentro de áreas de trabalho. Cada área de trabalho pode conter quantos pacotes você desejar. Para este seção, seu espaço de trabalho é denominado `ros2_ws`. Assim, a estrutura geral ficaria assim:

```bash
ros2_ws/
    src/
        my_package/
            package.xml
            setup.py
            ...
        my_package_2/
            package.xml
            setup.py
            ...
        my_package_x/
            package.xml
            setup.py
            ...
```

## Compilar um pacote
Quando você cria um pacote, você precisa compilá-lo para fazê-lo funcionar.

O comando a seguir irá compilar todo o seu diretório src e precisa ser emitido dentro do diretório inicial de um espaço de trabalho para funcionar (`ros2_ws`).
```bash
cd ~/ros2_ws
colcon build
```
> NUNCA EXECUTE ESSE COMANDO DENTRO DO DIRETÓRIO ros2_ws/src!!

Vá para o diretório `ros2_ws` e compile sua pasta de origem. Você pode fazer isso digitando o seguinte:

```bash
cd ~/ros2_ws
colcon build
```
Além disso, após a compilação, você deve criar o espaço de trabalho para garantir que as `modifications/updates` mais recentes sejam levadas em consideração:

```bash
cd ~/ros2_ws
source install/setup.bash
```

Às vezes (para grandes projetos), você não vai querer compilar todos os seus pacotes. Isso levaria muito tempo. Você pode usar o seguinte comando para compilar apenas os pacotes nos quais você fez alterações:
```bash
colcon build --packages-select <package_name>
```
Este comando apenas compilará os pacotes especificados e suas dependências.

Agora, tente compilar seu pacote chamado my_package com este comando.
```bash
colcon build --packages-select my_package
```
Para confirmar que seu pacote foi criado com sucesso, use alguns comandos do ROS relacionados a pacotes. Por exemplo, digite o seguinte:
```bash
source ~/ros2_ws/install/setup.bash
ros2 pkg list
ros2 pkg list | grep my_package
```

* `source ~/ros2_ws/install/setup.bash`: Nós fornecemos o ros2_ws para que o ROS2 também procure dentro deste espaço de trabalho por qualquer pacote.

* `ros2 pkg list`: Fornece uma lista com todos os pacotes em seu sistema ROS2.

* `ros2 pkg list | grep my_package`: Filtra, de todos os pacotes localizados no sistema ROS2, qualquer pacote que contenha a `string my_package`.

## O comando colcon_cd
Você também pode confirmar se seu pacote está lá usando o comando colcon_cd. O comando colcon_cd permite mover rapidamente para um determinado pacote.

Por exemplo, executando o comando abaixo:
```bash
colcon_cd my_package
```
Deve levá-lo para a pasta do seu pacote chamada `my_package`.

No entanto, este comando não está disponível por padrão no ROS. Para ativá-lo, você precisa obter o script de shell `colcon_cd.sh`, que é fornecido pelo pacote `colcon-cd`.

```bash
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/home/user/ros2_ws" >> ~/.bashrc
. ~/.bashrc
```
Você também pode usar o comando colcon_cd sem especificar nenhum pacote, assim:
```bash
colcon_cd
```
Este comando o levará à raiz do seu workapce. Neste caso, é `/home/user/ros2_ws`.

Você pode ler mais detalhes sobre como configurar `colcon_cd` em seu computador local aqui: [COLCON SETUP PAGE](https://colcon.readthedocs.io/en/released/user/installation.html#quick-directory-changes)

## Compreendendo o arquivo setup.py
Para que o colcon encontre os arquivos de inicialização, você precisa informar as ferramentas de configuração do Python sobre seus arquivos de inicialização usando o parâmetro `data_files` de `setup.py`.

Para fazer isso, você precisa adicionar algumas linhas ao seu arquivo setup.py. Essas linhas já estão no arquivo.
```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='somebody very awesome',
    maintainer_email='user@user.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple = my_package.simple:main'
        ],
    },
)
```
O principal objetivo deste código é adicionar um ponto de entrada ao script que você criou há pouco. Para fazer isso, trabalhe com um dicionário chamado `entry_points`. Dentro dele, você encontra um `array` chamado `console_scripts`. Esta é a informação do nó para gerar o executável.

```python
import os
from glob import glob
from setuptools import setup

package_name = 'my_package'


setup(
    
    #code
    ...
    #code
    
    entry_points={
            'console_scripts': [
                'simple = my_package.simple:main'
            ],
        },
    
    #code
    ...
    
)
```
Com essas linhas, você está adicionando um ponto de entrada ao script que escreveu anteriormente `simple.py`. Você pode ver esta linha da seguinte maneira:
```python
'<executable_name> = <package_name>.<script_name>:main'
```
Esta linha significa: execute a função `main()` dentro do script (os arquivos Python sem o `.py`) dentro do pacote criado com `ros2 pkg create --build-type ament_python`. E isso recebe um nome executável. Para manter as coisas limpas, uma estratégia que você pode seguir aqui (e nos arquivos de inicialização criados) é nomear o executável da mesma forma que o arquivo Python com `_exe` adicionado no final. Assim, no exemplo acima, seria
```python
'simple_exe = my_package.simple:main'
```
Observe que o nome do executável já é usado ao escrever o código para seu arquivo de inicialização.

Você também pode analisar outras linhas importantes do código `setup.py`. Então, vamos continuar com o array `data_files`.

```python
import os
from glob import glob
from setuptools import setup

package_name = 'my_package'


setup(
    
    #code
    ...
    #code
    
    data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
        ],
    
    #code
    ...
    #code
    
)
```
O objetivo deste código é instalar os arquivos de inicialização. Por exemplo, com o pacote chamado my_package, isso instalará todos os arquivos de inicialização da pasta `launch/`, em `~/ros2_ws/install/my_package/share/my_package/`.

O método `glob()` é usado para procurar arquivos com uma determinada estrutura ou nome em um determinado caminho.

```python
import glob
# If this is executed in a folder with a file named test0.py, test7, test11.py, bob.py
# It will print only the test0.py, test7.py
for py in glob.glob("test[0-9].py"):
    print(py)
```
No nosso caso, usando o `glob('launch/*.launch.py')`, estamos informando para procurar na pasta de inicialização todos os arquivos que terminam com a extensão `.launch.py`.

## Setuptools

Então, de onde vêm esses arquivos `setup.py` e `setup.cfg`? A resposta é a biblioteca `setuptools` do Python. Ele é completo, mantido ativamente, estável e foi projetado para empacotar projetos Python com mais facilidade.

Isso é perfeito ao trabalhar com arquivos Python no ROS2, então é por isso que esses arquivos são criados agora quando você especifica a criação de um pacote com `--build-type ament_python`.

Portanto, para resumir, o ROS2 usa a distribuição de módulos padrão do Python, que por sua vez usa ferramentas de configuração. É por isso que a função `setup()` é importada:

```python
from setuptools import setup
```

O arquivo `setup.py` é então análogo ao arquivo `C++` `CMakeLists.txt`.

Quando você cria um pacote ROS2 em Python, os seguintes parâmetros são incluídos automaticamente:

* **name**: O nome do seu pacote. Essencial.
* **version**: Versão do seu pacote.
* **data_files**: Lista de strings que especificam os arquivos de dados a serem instalados. É aqui que o package.xml e pastas como launch e config são adicionados para que possam ser encontrados pelo ROS2. Essencial.
* **install_requires**: Lista de strings que especificam quais outras distribuições devem ser instaladas quando o pacote é instalado, ou seja, as dependências do pacote. ferramentas de configuração sempre serão incluídas. Essencial.
* **zip_safe**: Booleano especificando se o projeto pode ser instalado e executado a partir do arquivo zip.
* **autor**
* **autor_email**
* **mantenedor**
* **e-mail_mantenedor**
* **palavras-chave**: String que fornece metadados descritivos.
classificadores**: String que descreve as categorias do pacote.
* **description**: Descreve o pacote em uma única linha.
* **licença**
* **entry_points**: Dicionário mapeando nomes de grupos de pontos de entrada para strings que definem os pontos de entrada, que são usados para dar suporte à descoberta dinâmica de serviços ou `plug-ins` fornecidos por um projeto (ou seja, ser capaz de digitar ros2 run `package_name` `executable_name` no terminal). Essencial.
Agora, se você der uma olhada no `setup.cfg`, verá que ele foi gerado com o seguinte:

```bash
[develop]
script_dir=$base/lib/my_package
[install]
install_scripts=$base/lib/my_package
```

Que indica onde os scripts serão instalados. Neste caso, `$base = ~/ros2_ws/install/my_package`. Se você navegar aqui, verá que os arquivos adicionados a `entry_points` aparecerão aqui.

Se você quiser ver a documentação completa do setuptools, dê uma olhada neste link: https://setuptools.pypa.io/en/latest/#. Você verá que é extremamente versátil e poderoso, mas isso está fora do escopo do ROS2. O ROS2 usa apenas uma fração do que esta biblioteca pode fazer.

## Symlink Install

Este argumento colcon: `--symlink-install` é a solução que o ROS2 dá para evitar ter que compilar, quando mudamos arquivos que não precisam de compilação, como arquivos `.py`, `.xml`, `.launch.py`, e assim por diante .

Essa é a teoria, na realidade existem muitos problemas em torno disso.

No momento em que escrevo isso, `--symlink-install` é útil apenas para `PYTHON SCRIPTS`.

Vejamos um exemplo que mostrará o status `--symlink-install`:

Vamos remover as pastas de compilação e compilar com e sem link simbólico.

Vamos criar um pacote para testar isso:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python build_test_pkg --dependencies rclpy 
cd build_test_pkg
mkdir params_files
touch params_files/params.yaml
mkdir launch
touch launch/test.launch.py
touch build_test_pkg/script1.py
```

> params.yaml

```yaml
robot_name: "Marceline"
serial_num: 123456789
```

> test.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='build_test_pkg',
            executable='script1_exe',
            output='screen'),
    ])
```

> script1.py

```python
import rclpy
# import the Node module from ROS2 python library
from rclpy.node import Node
import os
from ament_index_python.packages import get_package_share_directory
import yaml

def open_yaml_file(yaml_path):
    with open(yaml_path, "r") as stream:
        contents = None
        try:
            contents = yaml.safe_load(stream)            
        except yaml.YAMLError as exc:
            print(exc)

    print(contents)

    return contents

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)

    package_description = "build_test_pkg"
    yaml_file_path = os.path.join(get_package_share_directory(package_description), "params_files", "params.yaml")

    print("Path to files=="+str(yaml_file_path))

    
    yaml_data = open_yaml_file(yaml_file_path)

    print("YAML DATA XXXXX="+str(yaml_data))

    rclpy.shutdown()

if __name__ == '__main__':
    main() #call the main function
```

> setup.py

```python
from setuptools import setup
import os
from glob import glob


package_name = 'build_test_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ('share/ament_index/resource_index/packages',
        #     ['resource/' + package_name]),
        # ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params_files'),
         glob('params_files/*.yaml')),
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
            'script1_exe = build_test_pkg.script1:main'
        ],
    },
)
```
Neste `setup.py` colocamos apenas nos `data_files`:

* Os arquivos dentro da pasta de inicialização.
* Os arquivos dentro da pasta `param_files`
Não estamos adicionando:

* o arquivo package.xml
* O arquivo `resource/build_test_pkg` para o `ament_indexing`.

Isso ocorre porque queremos a quantidade mínima de arquivos presentes.

### Construir sem Symlink Install

Construiremos apenas o `build_test_pkg` para evitar outros pacotes e arquivos que possam tornar este exemplo mais complicado de seguir.

```bash
# Now we remove all the build an dinstall folders and compile
cd ~/ros2_ws/
rm -rf build install log
colcon build --packages-select build_test_pkg
source install/setup.bash
```
Vamos dar uma olhada nas pastas install e build

```bash
cd ~/ros2_ws/
ll install
```

> Output

```bash
drwxr-xr-x 3 user user  4096 Feb 10 10:35 ./
drwxr-xr-x 6 user user  4096 Feb 10 10:35 ../
-rw-r--r-- 1 user user     9 Feb 10 10:35 .colcon_install_layout
-rw-r--r-- 1 user user     0 Feb 10 10:35 COLCON_IGNORE
-rw-r--r-- 1 user user 13537 Feb 10 10:35 _local_setup_util_ps1.py
-rw-r--r-- 1 user user 13621 Feb 10 10:35 _local_setup_util_sh.py
drwxr-xr-x 4 user user  4096 Feb 10 10:35 build_test_pkg/
-rw-r--r-- 1 user user  3331 Feb 10 10:35 local_setup.bash
-rw-r--r-- 1 user user  2008 Feb 10 10:35 local_setup.ps1
-rw-r--r-- 1 user user  3721 Feb 10 10:35 local_setup.sh
-rw-r--r-- 1 user user  3726 Feb 10 10:35 local_setup.zsh
-rw-r--r-- 1 user user  1140 Feb 10 10:35 setup.bash
-rw-r--r-- 1 user user  1167 Feb 10 10:35 setup.ps1
-rw-r--r-- 1 user user  1903 Feb 10 10:35 setup.sh
-rw-r--r-- 1 user user  1128 Feb 10 10:35 setup.zsh
```

```bash
cd ~/ros2_ws/
ll build/build_test_pkg
```

> Output

```bash
total 36
drwxr-xr-x 4 user user 4096 Feb 10 11:46 ./
drwxr-xr-x 3 user user 4096 Feb 10 11:46 ../
drwxr-xr-x 3 user user 4096 Feb 10 11:46 build/
drwxr-xr-x 2 user user 4096 Feb 10 11:46 build_test_pkg.egg-info/
-rw-r--r-- 1 user user    2 Feb 10 11:46 colcon_build.rc
-rw-r--r-- 1 user user   65 Feb 10 11:46 colcon_command_prefix_setup_py.sh
-rw-r--r-- 1 user user 4274 Feb 10 11:46 colcon_command_prefix_setup_py.sh.env
-rw-r--r-- 1 user user 1517 Feb 10 11:46 install.log
```

Aqui temos apenas esta pasta `build/build_test_pkg/build`, sem `softlink`. Dentro desta pasta, podemos encontrar no final uma cópia HARD do script python `script1.py`:

```bash
ls build/build_test_pkg/build/lib/build_test_pkg/
```

### Construir com Symlink Install

```bash
# Now we remove all the build an dinstall folders and compile
cd ~/ros2_ws/
rm -rf build install log
colcon build --symlink-install --packages-select build_test_pkg 
source install/setup.bash
```
Vamos dar uma olhada nas pastas install e build
```bash
cd ~/ros2_ws/
ll install
```

Na pasta de instalação não há `softlinks`, com uma pequena exceção do `package.xml` em uma pasta perdida. Todos os arquivos encontrados aqui, MESMO COM SYMLINK, são cópias impressas dos que estão na pasta SOURCE.

> Output

```bash
drwxr-xr-x 3 user user  4096 Feb 10 10:44 ./
drwxr-xr-x 6 user user  4096 Feb 10 10:44 ../
-rw-r--r-- 1 user user     9 Feb 10 10:44 .colcon_install_layout
-rw-r--r-- 1 user user     0 Feb 10 10:44 COLCON_IGNORE
-rw-r--r-- 1 user user 13537 Feb 10 10:44 _local_setup_util_ps1.py
-rw-r--r-- 1 user user 13621 Feb 10 10:44 _local_setup_util_sh.py
drwxr-xr-x 4 user user  4096 Feb 10 10:44 build_test_pkg/
-rw-r--r-- 1 user user  3331 Feb 10 10:44 local_setup.bash
-rw-r--r-- 1 user user  2008 Feb 10 10:44 local_setup.ps1
-rw-r--r-- 1 user user  3721 Feb 10 10:44 local_setup.sh
-rw-r--r-- 1 user user  3726 Feb 10 10:44 local_setup.zsh
-rw-r--r-- 1 user user  1140 Feb 10 10:44 setup.bash
-rw-r--r-- 1 user user  1167 Feb 10 10:44 setup.ps1
-rw-r--r-- 1 user user  1903 Feb 10 10:44 setup.sh
-rw-r--r-- 1 user user  1128 Feb 10 10:44 setup.zsh
```
E os arquivos executados são os que estão na pasta de inicialização. Isso significa que não importa se você alterar os `param_files` ou iniciar arquivos na fonte, se NÃO compilar, estará usando as versões antigas deles da última compilação.

```bash
cd ~/ros2_ws/
ll build/build_test_pkg/
```

> Output

```bash
drwxr-xr-x 6 user user 4096 Feb 10 10:44 ./
drwxr-xr-x 3 user user 4096 Feb 10 10:44 ../
lrwxrwxrwx 1 user user   69 Feb 10 10:44 build_test_pkg -> /home/user/ros2_ws/src/ros2_build_tests/build_test_pkg/build_test_pkg/
drwxr-xr-x 2 user user 4096 Feb 10 10:44 build_test_pkg.egg-info/
-rw-r--r-- 1 user user    2 Feb 10 10:44 colcon_build.rc
-rw-r--r-- 1 user user   65 Feb 10 10:44 colcon_command_prefix_setup_py.sh
-rw-r--r-- 1 user user 4260 Feb 10 10:44 colcon_command_prefix_setup_py.sh.env
drwxr-xr-x 2 user user 4096 Feb 10 10:44 launch/
drwxr-xr-x 2 user user 4096 Feb 10 10:44 params_files/
lrwxrwxrwx 1 user user   64 Feb 10 10:44 setup.cfg -> /home/user/ros2_ws/src/ros2_build_tests/build_test_pkg/setup.cfg
lrwxrwxrwx 1 user user   63 Feb 10 10:44 setup.py -> /home/user/ros2_ws/src/ros2_build_tests/build_test_pkg/setup.py
drwxr-xr-x 3 user user 4096 Feb 10 10:44 share/
```

Aqui temos soft links, mas não são usados na execução.

```bash
cd ~/ros2_ws/
ll build/build_test_pkg/launch/
```

> Output

```bash
drwxr-xr-x 2 user user 4096 Feb 10 10:44 ./
drwxr-xr-x 6 user user 4096 Feb 10 10:44 ../
lrwxrwxrwx 1 user user   76 Feb 10 10:44 test.launch.py -> /home/user/ros2_ws/src/ros2_build_tests/build_test_pkg/launch/test.launch.py
```

Aqui você pode ver o softlink para os arquivos de inicialização

```bash
cd ~/ros2_ws/
ll build/build_test_pkg/params_files/
```

Aqui você pode ver o softlink para os arquivos de parâmetro

```bash
drwxr-xr-x 2 user user 4096 Feb 10 10:44 ./
drwxr-xr-x 6 user user 4096 Feb 10 10:44 ../
lrwxrwxrwx 1 user user   79 Feb 10 10:44 params.yaml -> /home/user/ros2_ws/src/ros2_build_tests/build_test_pkg/params_files/params.yaml
```

## Caso especial: scripts Python

Os scripts python são um caso especial. Eles são os únicos que se beneficiam da instalação do link simbólico. O motivo é que eles estão usando o sistema de gerenciamento de pacotes `setup.py` e alguma geração inteligente de arquivos para poder usar a funcionalidade de `softlink`.

Quando compilamos, os scripts python para os quais definimos um ponto de entrada no `setup.py` gerarão este arquivo na pasta de instalação:

```bash
cat install/build_test_pkg/lib/build_test_pkg/script1_exe
```

> Output

```bash
#!/usr/bin/python3
# EASY-INSTALL-ENTRY-SCRIPT: 'build-test-pkg','console_scripts','script1_exe'
__requires__ = 'build-test-pkg'
import re
import sys
from pkg_resources import load_entry_point

if __name__ == '__main__':
    sys.argv[0] = re.sub(r'(-script\.pyw?|\.exe)?$', '', sys.argv[0])
    sys.exit(
        load_entry_point('build-test-pkg', 'console_scripts', 'script1_exe')()
    )
```

Este é o script executado quando você executa o comando `ros2 run build_test_pkg script1_exe`.

Vamos ver o que esse script usa em ambos os casos COM ou SEM link simbólico:

### Sem Symlink Install

```bash
cd ~/ros2_ws/
rm -rf build install log
colcon build --packages-select build_test_pkg
source install/setup.bash
ll build/build_test_pkg/
```

> Output

```bash
...
drwxr-xr-x 3 user user 4096 Feb 10 11:14 build/
...
```

O script python é copiado para esta pasta de construção. Este é o arquivo que o `install/build_test_pkg/lib/build_test_pkg/script1_exe` mostrado anteriormente obterá. É por isso que as alterações feitas no script python de origem `script1.py` não serão refletidas até que você recompile.

```bash
ls build/build_test_pkg/build/lib/build_test_pkg/
```

> Output

```bash
__init__.py  script1.py
```

Como você pode ver, é apenas uma cópia normal do arquivo, sem softlink.

### Com Symlink Install

```bash
cd ~/ros2_ws/
rm -rf build install log
colcon build --symlink-install --packages-select build_test_pkg 
source install/setup.bash
ll build/build_test_pkg/
```

> Output

```bash
..
lrwxrwxrwx 1 user user   69 Feb 10 11:11 build_test_pkg -> /home/user/ros2_ws/src/ros2_build_tests/build_test_pkg/build_test_pkg/
..
```

Aqui você pode ver que agora, entre outras coisas, criamos um softlink para a pasta do módulo python `/home/user/ros2_ws/src/ros2_build_tests/build_test_pkg`.

Isso significa que agora quando executamos o `install/build_test_pkg/lib/build_test_pkg/script1_exe` através do comando `ros2 run build_test_pkg script1_exe` , ele obtém o softlink e, portanto, TODAS as alterações feitas no `script1.py` serão refletidas SEM TER QUE COMPILAR.

Para ver que realmente este `install/build_test_pkg/lib/build_test_pkg/script1_exe` é o responsável por executar o ponto de entrada `script1_exe`, vamos:

Execute-o primeiro e veja se funciona corretamente.
Remova esse executável e tente executá-lo.

```bash
# Compile with or without softlink , doesn't matter
cd ~/ros2_ws/
rm -rf build install log
colcon build --symlink-install --packages-select build_test_pkg 
source install/setup.bash
# We execute
ros2 run build_test_pkg script1_exe
```

> Output

```bash
Path to files==/home/user/ros2_ws/install/build_test_pkg/share/build_test_pkg/params_files/params.yaml
{'robot_name': 'Marceline', 'serial_num': 123456789}
YAML DATA YYYY={'robot_name': 'Marceline', 'serial_num': 123456789}
```

```bash
cd ~/ros2_ws/install/build_test_pkg/lib/build_test_pkg
rm -rf script1_exe
ros2 run build_test_pkg script1_exe
```

> Output

```bash
No executable found
```
Como você pode ver, sem esse arquivo `script1_exe`, nada funciona.

## Pastas especiais dentro de pacotes ROS2

Existem duas pastas sobre as quais ainda não falamos e que você pode encontrar em todos os pacotes criados pelo ROS2:

* resource
* test
Vejamos para que servem:

### Pasta `resource`
O "índice de pacotes" faz parte do índice de recursos de ament. É assim que as ferramentas do ROS 2 descobrem quais pacotes estão instalados.

Vamos dar uma olhada primeiro no que você pode encontrar dentro dele por padrão. Vamos usar o pacote de exemplo da seção anterior `build_test_pkg`:

```bash
# Compile with or without softlink , doesn't matter
cd ~/ros2_ws/
rm -rf build install log
colcon build --symlink-install --packages-select build_test_pkg 
source install/setup.bash
# Have a look at the contents
ls src/build_test_pkg/resource/
```

E este arquivo está totalmente vazio. Por que?

Bem, isso ocorre porque é usado pelo índice de recursos de ament. É usado pelo ROS2 para saber quais pacotes estão instalados no espaço de trabalho.

```bash
ls install/build_test_pkg/share/ament_index/resource_index/packages/
```
> Output

```bash
build_test_pkg
```

Você pode ver que aqui esse arquivo é colocado na pasta de instalação quando compilamos, dando acesso ao ROS para encontrar esse pacote. Se, por exemplo, removermos esse arquivo, o ROS2 não conseguirá encontrar o pacote:

```bash

cd ~/ros2_ws/
rm -rf install/build_test_pkg/share/ament_index/resource_index/packages/build_test_pkg
ros2 run build_test_pkg script1_exe
```

> Output

```bash
Package 'build_test_pkg' not found
```
E quem gera essa estrutura? Bem, isso é feito automaticamente, você não precisa editar o `setup.py` nem nada. É feito cada vez que construímos:

```bash
cd ~/ros2_ws/
colcon build --symlink-install --packages-select build_test_pkg 
source install/setup.bash
ros2 run build_test_pkg script1_exe
```

> Output

```bash
Path to files==/home/user/ros2_ws/install/build_test_pkg/share/build_test_pkg/params_files/params.yaml
{'robot_name': 'Marceline', 'serial_num': 123456789}
YAML DATA YYYY={'robot_name': 'Marceline', 'serial_num': 123456789}
```

> Então, se isso é feito automaticamente sempre, por que se preocupar em explicar o que são? Porque em versões futuras do ROS2 teremos que adicioná-lo ao arquivo setup.py porque NÃO SERÁ feito automaticamente. É por isso que no setup.py padrão temos esta linha:

```python
('share/ament_index/resource_index/packages',['resource/' + package_name]),
```

Esta linha está definindo esse pacote como um recurso.

Esta pasta de recursos "registra" metainformações sobre os recursos instalados pelos pacotes, usando uma classificação de pasta. Isso oferece algumas vantagens que giram em torno da ideia de:

* Evitando o rastreamento de arquivos para encontrar pacotes. Porque o ROS2 tem uma pasta muito específica onde procurar esses tipos de recursos.
* Isso o torna muito mais rápido e também evita colisões de pacotes.

### Pasta `test`

Como o nome indica, esta pasta é para colocar todos os seus testes de unidade, testes de integração e muito mais. Isso foi criado para padronizar e promover que todos os seus pacotes ROS2 tenham scripts de teste. Isso é vital, especialmente se você deseja disponibilizar seus pacotes para a comunidade ou para o pool de pacotes do ROS2.

Existem dois comandos que permitem executar os testes internos.

* `launch_test`: Este é o melhor para executar scripts de testes únicos e ver todas as mensagens de saída e depurar seus scripts de testes. Recomendado para a fase de desenvolvimento de seus testes.

* `colcon test`: Este é um comando mais global. Ele irá executar TODOS os testes dentro da pasta test dos pacotes. Mas você pode usar o argumento `packages-select` para executar apenas um único pacote de testes.

Por padrão, todos os pacotes criados para `ament_python` possuem estes testes:

* test_copyright.py
* test_flake8.py
* test_pep257.py

Vamos removê-los por enquanto porque eles fornecem um aviso de problema do ROS2 que interferiria na visualização de erros em nossos testes:

```bash
Warning: Using or importing the ABCs from 'collections' instead of from 'collections.abc' is deprecated since Python 3.3, and in 3.9it will stop working
```
Agora vamos criar nosso próprio teste.
```bash
# Compile with or without softlink , doesn't matter
cd ~/ros2_ws/
touch src/build_test_pkg/test/check_dummy_scan_msgs_test.py
touch src/build_test_pkg/test/dummy_laser.py
rm -rf src/build_test_pkg/test/test_copyright.py
rm -rf src/build_test_pkg/test/test_flake8.py
rm -rf src/build_test_pkg/test/test_pep257.py
rm -rf build install log
colcon build --symlink-install --packages-select build_test_pkg 
source install/setup.bash
```

> dummy_laser.py

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

import random


class DummyLaser(Node):

    def __init__(self):
        super().__init__('DummyLaser')
        self.count = 0
        self.publisher = self.create_publisher(LaserScan, '/dummy_scan', 10)
        self.timer = self.create_timer(1.0, self.callback)

    def callback(self):
        data = random.random()
        data2 = random.random()
        msg = LaserScan()
        msg.ranges = [data, data2]
        self.get_logger().info('Publishing: '+str((msg.ranges)))
        self.publisher.publish(msg)
        self.count += 1


def main(args=None):
    rclpy.init(args=args)

    node = DummyLaser()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```


> check_dummy_scan_msgs_test.py

```python
import os
import sys

from threading import Event
from threading import Thread
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    path_to_test = os.path.dirname(__file__)

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            executable=sys.executable,
            arguments=[os.path.join(path_to_test, 'dummy_laser.py')],
            name='testing_node_dummy_scan',
        ),

        launch_testing.actions.ReadyToTest()
    ])


class TestFixture(unittest.TestCase):

    def test_check_if_msgs_published(self, proc_output):
        rclpy.init()
        try:
            node = MakeTestNode('test_node')
            node.start_subscriber()
            msgs_received_flag = node.msg_event_object.wait(timeout=5.0)
            assert msgs_received_flag, 'ERROR in TEST: Did not get any message!'
        finally:
            rclpy.shutdown()


class MakeTestNode(Node):

    def __init__(self, name='test_node'):
        super().__init__(name)
        self.msg_event_object = Event()

    def start_subscriber(self):
        # Create a subscriber
        self.subscription = self.create_subscription(
            LaserScan,
            '/dummy_scan',
            self.subscriber_callback,
            10
        )

        # Add a spin thread
        self.ros_spin_thread = Thread(
            target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def subscriber_callback(self, data):
        self.msg_event_object.set()
```

O decorador `< b>@pytest.mark.launch_test</ b>` é aquele que permite que colcon test saiba que é um teste.

Compilar:

```bash
cd ~/ros2_ws/
colcon build --symlink-install --packages-select build_test_pkg 
source install/setup.bash
```
Agora vamos executá-lo das duas maneiras:

```bash
launch_test ~/ros2_ws/src/build_test_pkg/test/check_dummy_scan_msgs_test.py
```

> Output

```bash
[INFO] [launch]: All log files can be found below /home/user/.ros/log/2023-04-18-05-12-01-200458-1_xterm-5211
[INFO] [launch]: Default logging verbosity is set to INFO
test_check_if_msgs_published (check_dummy_scan_msgs_test.TestFixture) ... [INFO] [python3-1]: process started with pid [5231]
[python3-1] [INFO] [1681794722.585024445] [testing_node_dummy_scan]: Publishing: array('f', [0.49915751814842224, 0.26657408475875854])
ok

----------------------------------------------------------------------
Ran 1 test in 1.364s

OK
[INFO] [python3-1]: sending signal 'SIGINT' to process[python3-1]
[python3-1] Traceback (most recent call last):
[python3-1]   File "/home/user/ros2_ws/src/build_test_pkg/test/dummy_laser.py", line 43, in 
[python3-1]     main()
[python3-1]   File "/home/user/ros2_ws/src/build_test_pkg/test/dummy_laser.py", line 39, in main
[python3-1]     rclpy.shutdown()
[python3-1]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 126, in shutdown
[python3-1]     _shutdown(context=context)
[python3-1]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/utilities.py", line 58, in shutdown
[python3-1]     return context.shutdown()
[python3-1]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/context.py", line 102, in shutdown
[python3-1]     self.__context.shutdown()
[python3-1] rclpy._rclpy_pybind11.RCLError: failed to shutdown: rcl_shutdown already called on the given context, at ./src/rcl/init.c:241
[ERROR] [python3-1]: process has died [pid 5231, exit code 1, cmd '/usr/bin/python3 /home/user/ros2_ws/src/build_test_pkg/test/dummy_laser.py --ros-args -r __node:=testing_node_dummy_scan'].

----------------------------------------------------------------------
Ran 0 tests in 0.000s

OK
```

Você pode ver que tudo correu bem, sem problemas. O único erro que ocorre está relacionado ao ROS2 matando o script. Nada com que se preocupar, pois o teste `testing_node_dummy_scan` foi executado.

Como esse comando compila, precisamos estar em `~/ros2_ws` para evitar a geração de pastas de instalação em todos os lugares.

```bash
cd ~/ros2_ws/
colcon test --packages-select build_test_pkg
```

> Output

```bash
Starting >>> build_test_pkg
--- stderr: build_test_pkg

=============================== warnings summary ===============================
test/check_dummy_scan_msgs_test.py::check_dummy_scan_msgs_test
test/check_dummy_scan_msgs_test.py::check_dummy_scan_msgs_test
  Warning: There is no current event loop

-- Docs: https://docs.pytest.org/en/stable/warnings.html
---
Finished <<< build_test_pkg [2.59s]

Summary: 1 package finished [2.79s]
  1 package had stderr output: build_test_pkg
```

Esse aviso está sempre lá. Está relacionado a este problema aqui, se você estiver interessado em [Pytest ISSUES](https://github.com/pytest-dev/pytest-asyncio/issues/212)

Agora, vamos fazer o teste falhar, não publicando a mensagem e, portanto, o teste deve falhar:

> dummy_laser.py ( with error )

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

import random


class DummyLaser(Node):

    def __init__(self):
        super().__init__('DummyLaser')
        self.count = 0

        self.publisher = self.create_publisher(LaserScan, '/dummy_scan', 10)
        self.timer = self.create_timer(1.0, self.callback)

    def callback(self):
        data = random.random()
        data2 = random.random()
        msg = LaserScan()
        msg.ranges = [data, data2]
        self.get_logger().info('Publishing: '+str((msg.ranges)))
        # We comment this and there fore noone will be publishing in the **/dummy_scan topic**
        # self.publisher.publish(msg)
        self.count += 1


def main(args=None):
    rclpy.init(args=args)

    node = DummyLaser()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```
Agora vamos executá-lo das duas maneiras:

```bash
launch_test ~/ros2_ws/src/build_test_pkg/test/check_dummy_scan_msgs_test.py
```

> Output

```bash
[INFO] [launch]: All log files can be found below /home/user/.ros/log/2023-04-18-05-20-08-173127-1_xterm-6057
[INFO] [launch]: Default logging verbosity is set to INFO
test_check_if_msgs_published (check_dummy_scan_msgs_test.TestFixture) ... [INFO] [python3-1]: process started with pid [6069]
[python3-1] [INFO] [1681795209.884354833] [testing_node_dummy_scan]: Publishing: array('f', [0.3999781012535095, 0.38128629326820374])
[python3-1] [INFO] [1681795210.866394735] [testing_node_dummy_scan]: Publishing: array('f', [0.2707482874393463, 0.7008478045463562])
[python3-1] [INFO] [1681795211.866443920] [testing_node_dummy_scan]: Publishing: array('f', [0.31215575337409973, 0.3785834014415741])
[python3-1] [INFO] [1681795212.866375460] [testing_node_dummy_scan]: Publishing: array('f', [0.20264501869678497, 0.37846311926841736])
FAIL

======================================================================
FAIL: test_check_if_msgs_published (check_dummy_scan_msgs_test.TestFixture)
----------------------------------------------------------------------
Traceback (most recent call last):
  File "/home/user/ros2_ws/src/build_test_pkg/test/check_dummy_scan_msgs_test.py", line 43, in test_check_if_msgs_published
    assert msgs_received_flag, 'ERROR in TEST: Did not get any message!'
AssertionError: ERROR in TEST: Did not get any message!

----------------------------------------------------------------------
Ran 1 test in 5.401s

FAILED (failures=1)
[INFO] [python3-1]: sending signal 'SIGINT' to process[python3-1]
[python3-1] Traceback (most recent call last):
[python3-1]   File "/home/user/ros2_ws/src/build_test_pkg/test/dummy_laser.py", line 45, in 
[python3-1]     main()
[python3-1]   File "/home/user/ros2_ws/src/build_test_pkg/test/dummy_laser.py", line 41, in main
[python3-1]     rclpy.shutdown()
[python3-1]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 126, in shutdown
[python3-1]     _shutdown(context=context)
[python3-1]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/utilities.py", line 58, in shutdown
[python3-1]     return context.shutdown()
[python3-1]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/context.py", line 102, in shutdown
[python3-1]     self.__context.shutdown()
[python3-1] rclpy._rclpy_pybind11.RCLError: failed to shutdown: rcl_shutdown already called on the given context, at ./src/rcl/init.c:241
[ERROR] [python3-1]: process has died [pid 6069, exit code 1, cmd '/usr/bin/python3 /home/user/ros2_ws/src/build_test_pkg/test/dummy_laser.py --ros-args -r __node:=testing_node_dummy_scan'].

----------------------------------------------------------------------
Ran 0 tests in 0.000s

OK
```
Aqui temos o erro de `assertion: AssertionError: ERROR in TEST: Did not get any message!`. Então parece que o teste funcionou.

```bash
colcon test --packages-select build_test_pkg
```

> Output

```bash
Starting >>> build_test_pkg
--- stderr: build_test_pkg
=============================== warnings summary ===============================
test/check_dummy_scan_msgs_test.py::check_dummy_scan_msgs_test
test/check_dummy_scan_msgs_test.py::check_dummy_scan_msgs_test
  Warning: There is no current event loop

-- Docs: https://docs.pytest.org/en/stable/warnings.html
---
Finished <<< build_test_pkg [6.28s]     [ with test failures ]

Summary: 1 package finished [6.47s]
  1 package had stderr output: build_test_pkg
  1 package had test failures: build_test_pkg
```

Agora você vê que temos um erro, mas não é muito preciso.