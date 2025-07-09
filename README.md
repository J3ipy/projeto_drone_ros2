# Projeto Drone Autônomo com ROS 2

<div align="center">
  <em>Controle, percepção e autonomia para o seu drone DJI Tello.</em>
</div>

<div align="center">

[![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)
[![Docker](https://img.shields.io/badge/Docker-2496ED?style=for-the-badge&logo=docker&logoColor=white)](https://www.docker.com/)
[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-0A3E63?style=for-the-badge&logo=ros&logoColor=white)](https://ros.org/)
[![Visual Studio Code](https://img.shields.io/badge/VS_Code-007ACC?style=for-the-badge&logo=visualstudiocode&logoColor=white)](https://code.visualstudio.com/)

</div>

---

## 📖 Visão Geral

Este projeto transforma um drone DJI Tello numa plataforma de voo autônoma utilizando o ecossistema **ROS 2**. Através de um ambiente de desenvolvimento containerizado com **Docker**, o projeto oferece uma arquitetura modular para controle, processamento de visão e gerenciamento de missões, permitindo que o drone navegue de forma inteligente usando códigos QR como pontos de referência.

### ✨ Funcionalidades Principais

* **🐳 Ambiente Containerizado:** Configuração simplificada e 100% reprodutível com Docker e VS Code Dev Containers. Esqueça os problemas de dependências!
* **🚁 Arquitetura Modular ROS 2:** Nós independentes para o driver do drone, processamento de visão e controle da missão, garantindo organização e escalabilidade.
* **📸 Visão Computacional em Tempo Real:** Um nó de visão dedicado detecta e decodifica códigos QR para guiar a navegação do drone.
* **🎯 Planejamento de Missão:** As missões são facilmente definidas num ficheiro `mission.json`, permitindo a criação de rotas de voo complexas sem alterar o código.
* **📡 Comunicação por Serviços e Tópicos:** Utiliza os padrões do ROS 2 para uma comunicação robusta e desacoplada entre os módulos do sistema.

---

## 🚀 Começando

Siga os passos abaixo para ter o ambiente de desenvolvimento a funcionar em poucos minutos.

### ✅ Pré-requisitos

Antes de começar, garanta que tem o seguinte software instalado na sua máquina:

1.  [**WSL2 (Windows Subsystem for Linux)**](https://learn.microsoft.com/pt-br/windows/wsl/install): Permite executar um ambiente Linux no Windows.
2.  [**Docker Desktop**](https://www.docker.com/products/docker-desktop/): A plataforma para criar e gerir os nossos contêineres. Certifique-se de que a **integração com WSL2** está ativa nas configurações.
3.  [**Visual Studio Code**](https://code.visualstudio.com/): O nosso editor de código.
4.  **Extensão Dev Containers:** Instale a extensão `ms-vscode-remote.remote-containers` diretamente no VS Code.

### 🛠️ Instalação e Execução

A forma mais simples de executar este projeto é através do ambiente de desenvolvimento containerizado.

1.  **Clone o repositório para a sua máquina:**
    ```bash
    git clone [https://github.com/J3ipy/projeto_drone_ros2.git](https://github.com/J3ipy/projeto_drone_ros2.git)
    ```

2.  **Abra o projeto no VS Code:**
    ```bash
    cd projeto_drone_ros2
    code .
    ```

3.  **Reabra no Contêiner:**
    * Assim que o VS Code abrir, ele detetará a pasta `.devcontainer` e mostrará uma notificação no canto inferior direito.
    * Clique em **"Reopen in Container"**.
    * Aguarde enquanto o VS Code constrói a imagem Docker e configura o ambiente de desenvolvimento. Este processo pode demorar alguns minutos na primeira vez.

4.  **Pronto!** O seu ambiente está 100% configurado. O terminal integrado do VS Code já está dentro do contêiner e com o ambiente ROS 2 ativado.

---

## 🕹️ Como Usar

Com o ambiente a funcionar, siga estes passos para ver o drone em ação:

1.  **Ligue o drone Tello.**
2.  No seu computador (Windows), **conecte-se à rede Wi-Fi criada pelo drone.**
3.  No terminal integrado do VS Code (que já está dentro do contêiner), execute o ficheiro de lançamento principal:
    ```bash
    ros2 launch tello_control mission_launch.py
    ```

O sistema será iniciado, o drone irá decolar e começará a executar a missão definida no `mission.json`.

---

## 🏗️ Estrutura do Projeto

O coração do projeto está dividido em três nós ROS 2 principais:

* `drone_node.py`: Funciona como o driver do drone. É responsável por enviar comandos (decolar, aterrar, mover), receber telemetria e publicar o feed de vídeo da câmara num tópico.
* `vision_node.py`: Subscreve ao tópico de vídeo, processa cada frame para detetar e decodificar códigos QR, e publica os dados do QR encontrado num novo tópico.
* `mission_controller_node.py`: O cérebro da operação. Subscreve aos dados do QR code, gere a máquina de estados da missão (procurar, navegar, etc.) e invoca os serviços do `drone_node` para executar as ações definidas no `mission.json`.

