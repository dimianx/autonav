# ========================
# Base Image
# ========================
FROM osrf/ros:humble-desktop-full as base
ENV ROS_DISTRO=humble
ENV ROS_WORKSPACE=/autonav_ws
SHELL ["/bin/bash", "-c"]

COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

# ========================
# Devel Image
# ========================
FROM base as devel

ARG USERNAME=devuser
ARG UID=1000
ARG GID=${UID}

RUN mkdir -p /autonav_ws/src
WORKDIR /autonav_ws

COPY ./autonav_navigation ./src/autonav_navigation
COPY ./autonav_gz ./src/autonav_gz
COPY ./autonav_rviz ./src/autonav_rviz

RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends --no-install-suggests \
    wget \
    gdb \
    gdbserver \
    nano \
    git \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /autonav_ws
RUN source /opt/ros/humble/setup.bash && \
    apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends --no-install-suggests \
    python3-rosdep \
    x11-apps \
    wget

RUN wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add - && \
    sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list' && \
    apt update

RUN wget https://raw.githubusercontent.com/clearpathrobotics/public-rosdistro/master/rosdep/50-clearpath.list \
        -O /etc/ros/rosdep/sources.list.d/50-clearpath.list && \
    rosdep update

RUN source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build && \
    source /autonav_ws/install/setup.bash

RUN rm -rf /var/lib/apt/lists/*

RUN groupadd --gid $GID $USERNAME \
    && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && mkdir -p /home/${USERNAME} \
    && chown -R ${UID}:${GID} /home/${USERNAME}

RUN chown -R ${UID}:${GID} /autonav_ws/

USER ${USERNAME}
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc