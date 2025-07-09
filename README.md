# Projeto Drone Autônomo com ROS 2

### Controle, percepção e autonomia para o seu drone DJI Tello.

📖 Visão Geral

Este projeto transforma um drone DJI Tello numa plataforma de voo autônoma utilizando o ecossistema ROS 2. Através de um ambiente de desenvolvimento containerizado com Docker, o projeto oferece uma arquitetura modular para controle, processamento de visão e gerenciamento de missões, permitindo que o drone navegue de forma inteligente usando códigos QR como pontos de referência.

---

✨ Funcionalidades Principais

🐳 Ambiente Containerizado: Configuração simplificada e 100% reprodutível com Docker e VS Code Dev Containers. Esqueça os problemas de dependências!

🚁 Arquitetura Modular ROS 2: Nós independentes para o driver do drone, processamento de visão e controle da missão, garantindo organização e escalabilidade.

📸 Visão Computacional em Tempo Real: Um nó de visão dedicado detecta e decodifica códigos QR para guiar a navegação do drone.

🎯 Planejamento de Missão: As missões são facilmente definidas num ficheiro mission.json, permitindo a criação de rotas de voo complexas sem alterar o código.

📡 Comunicação por Serviços e Tópicos: Utiliza os padrões do ROS 2 para uma comunicação robusta e desacoplada entre os módulos do sistema.


---

🚀 Começando

Siga os passos abaixo para ter o ambiente de desenvolvimento a funcionar em poucos minutos.

✅ Pré-requisitos

Antes de começar, garanta que tem o seguinte software instalado na sua máquina:

WSL2 (Windows Subsystem for Linux): Permite executar um ambiente Linux no Windows.

Docker Desktop: A plataforma para criar e gerir os nossos contêineres. Certifique-se de que a integração com WSL2 está ativa nas configurações.

Visual Studio Code: O nosso editor de código.

Extensão Dev Containers: Instale a extensão ms-vscode-remote.remote-containers diretamente no VS Code.

🛠️ Instalação e Execução

A forma mais simples de executar este projeto é através do ambiente de desenvolvimento containerizado.

Clone o repositório para a sua máquina:

Bash

git clone https://github.com/J3ipy/projeto\_drone\_ros2.git

Abra o projeto no VS Code:

Bash

cd projeto\_drone\_ros2

code .

Reabra no Contêiner:

Assim que o VS Code abrir, ele detetará a pasta .devcontainer e mostrará uma notificação no canto inferior direito.

Clique em "Reopen in Container".

Aguarde enquanto o VS Code constrói a imagem Docker e configura o ambiente de desenvolvimento. Este processo pode demorar alguns minutos na primeira vez.

Pronto! O seu ambiente está 100% configurado. O terminal integrado do VS Code já está dentro do contêiner e com o ambiente ROS 2 ativado.

🕹️ Como Usar

Com o ambiente a funcionar, siga estes passos para ver o drone em ação:

Ligue o drone Tello.

No seu computador (Windows), conecte-se à rede Wi-Fi criada pelo drone.

No terminal integrado do VS Code (que já está dentro do contêiner), execute o ficheiro de lançamento principal:

Bash

ros2 launch tello\_control mission\_launch.py

O sistema será iniciado, o drone irá decolar e começará a executar a missão definida no mission.json.

🏗️ Estrutura do Projeto

O coração do projeto está dividido em três nós ROS 2 principais:

drone\_node.py: Funciona como o driver do drone. É responsável por enviar comandos (decolar, aterrar, mover), receber telemetria e publicar o feed de vídeo da câmara num tópico.

vision\_node.py: Subscreve ao tópico de vídeo, processa cada frame para detetar e decodificar códigos QR, e publica os dados do QR encontrado num novo tópico.

mission\_controller\_node.py: O cérebro da operação. Subscreve aos dados do QR code, gere a máquina de estados da missão (procurar, navegar, etc.) e invoca os serviços do drone\_node para executar as ações definidas no mission.json.

📄 Licença

Este projeto está licenciado sob a Licença Apache 2.0. Veja o ficheiro LICENSE para mais detalhes.
