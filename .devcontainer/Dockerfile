# Começamos com uma imagem oficial do ROS 2 Humble que inclui ferramentas básicas
FROM ros:humble-ros-base

# Define argumentos para o nome de utilizador (padrão é 'vscode')
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Evita que os comandos peçam inputs interativos durante a construção
ENV DEBIAN_FRONTEND=noninteractive

# Instala as dependências do sistema e do projeto
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-opencv \
    ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# Instala a biblioteca do Tello via pip
RUN pip3 install djitellopy "numpy<2.0"

# Cria um utilizador não-root para segurança e boas práticas
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# --- CONFIGURAÇÃO GLOBAL DO AMBIENTE ROS ---
# Adiciona os scripts de ativação aos perfis globais do shell.
# Isto garante que qualquer shell (sh, bash, interativo ou não) carregue o ambiente.
RUN echo "source /opt/ros/humble/setup.bash" >> /home/vscode/.bashrc && \
    echo "if [ -f /home/vscode/ros2_ws/install/setup.bash ]; then source /home/vscode/ros2_ws/install/setup.bash; fi" >> /home/vscode/.bashrc

# Define a variável de ambiente BASH_ENV para forçar o carregamento em shells não interativos
ENV BASH_ENV /etc/bash.bashrc

# Muda para o utilizador não-root
USER $USERNAME

# Cria o diretório do workspace ROS 2
RUN mkdir -p /home/$USERNAME/ros2_ws/src

# Define o diretório de trabalho padrão
WORKDIR /home/$USERNAME/ros2_ws