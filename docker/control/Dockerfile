# ---OLD VERSION---
#FROM hydronautics/serial:humble

#RUN apt update && apt install -y --no-install-recommends graphviz graphviz-dev python3-pip
#RUN pip install transitions[diagrams]

# WORKDIR /stingray
# COPY . /stingray
# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && source /additional_packages/install/setup.bash && source /stingray_core/install/setup.bash && colcon build --packages-select stingray_devices stingray_interfaces stingray_launch stingray_missions stingray_movement stingray_utils"

# COPY docker/control/entrypoint.sh /entrypoint.sh
# RUN chmod +x  /entrypoint.sh
# ENTRYPOINT ["/entrypoint.sh"]

# RUN echo 'source /stingray/install/setup.bash' >> /root/.bashrc

#CMD ["bash"]


FROM ros:humble-ros-base

# Установим нужные пакеты + утилиты для подсветки терминала + RViz2
RUN apt-get update && apt-get install -y \
    python3-pip python3-colcon-common-extensions \
    libasio-dev netcat socat \
    ros-humble-rclcpp ros-humble-std-msgs ros-humble-sensor-msgs ros-humble-udp-msgs \
    bash bash-completion command-not-found less \
    graphviz \
    graphviz-dev \
    python3-colorama \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-gui-py \
    ros-humble-robot-localization \
    # Добавляем RViz2 и его зависимости
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-visualization-msgs \
    # GUI зависимости для работы в Docker
    libxcb-xinerama0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-xfixes0 \
    libxcb-xv0 \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

RUN pip install transitions[diagrams]

# Настройка цветного вывода и подсветки
RUN echo 'export TERM=xterm-256color' >> ~/.bashrc && \
    echo 'force_color_prompt=yes' >> ~/.bashrc && \
    echo 'export LS_OPTIONS="--color=auto"' >> ~/.bashrc && \
    echo 'eval "`dircolors`"' >> ~/.bashrc && \
    echo 'alias ls="ls $LS_OPTIONS"' >> ~/.bashrc && \
    echo 'alias ll="ls $LS_OPTIONS -l"' >> ~/.bashrc && \
    echo 'alias l="ls $LS_OPTIONS -lA"' >> ~/.bashrc && \
    echo 'alias grep="grep --color=auto"' >> ~/.bashrc && \
    echo 'alias ip="ip --color=auto"' >> ~/.bashrc && \
    echo 'export GCC_COLORS="error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01"' >> ~/.bashrc

# Цветной prompt с информацией о контейнере
RUN echo 'export PS1="\[\033[1;32m\]\u@stingray-container:\w\$\[\033[0m\] "' >> ~/.bashrc

# Настройка для подсветки синтаксиса в редакторах (опционально)
RUN echo 'set -o vi' >> ~/.bashrc && \
    echo 'set editing-mode vi' >> ~/.inputrc && \
    echo 'set show-mode-in-prompt on' >> ~/.inputrc

# Добавляем алиас для запуска RViz2 с правильными параметрами
RUN echo 'alias rviz2="rviz2 -d /stingray/config.rviz"' >> ~/.bashrc && \
    echo 'alias rviz2-default="rviz2"' >> ~/.bashrc

# Установка переменных окружения для цветного вывода и GUI
ENV TERM=xterm-256color
ENV PYTHONUNBUFFERED=1
ENV FORCE_COLOR=1
ENV COLCON_DEFAULT_OPTS="--event-handlers console_cohesion+"
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:0

# Рабочая директория
WORKDIR /stingray

RUN groupadd -f i2c && usermod -aG i2c root

COPY docker/control/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
