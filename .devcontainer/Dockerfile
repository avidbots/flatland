ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=flatland

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    ccache \
    gdb \
    git \
    openssh-client \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    sudo \
    && rm -rf /var/lib/apt/lists/*

COPY . /tmp/flatland
RUN mkdir -p /opt/flatland_upstream_ws/src && \
        vcs import /opt/flatland_upstream_ws/src < /tmp/flatland/flatland-not-released.repos && \
        rosdep update && \
    apt-get update && \
        rosdep install --from-paths /tmp/flatland /opt/flatland_upstream_ws/src \
            --ignore-src --rosdistro "$ROS_DISTRO" -y && \
        . "/opt/ros/$ROS_DISTRO/setup.sh" && \
        colcon --log-base /opt/flatland_upstream_ws/log build \
            --base-paths /opt/flatland_upstream_ws/src \
            --build-base /opt/flatland_upstream_ws/build \
            --install-base /opt/flatland_upstream_ws/install && \
    rm -rf /tmp/flatland /var/lib/apt/lists/*

ENV CCACHE_DIR=/tmp/.ccache
ENV QT_QPA_PLATFORM=xcb
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
WORKDIR /opt/overlay_ws
RUN if id -u ubuntu >/dev/null 2>&1; then \
        if [ "$USERNAME" != ubuntu ]; then \
            usermod -l "$USERNAME" -d "/home/$USERNAME" -m ubuntu && \
            groupmod -n "$USERNAME" ubuntu; \
        fi; \
    else \
        groupadd -g 1000 "$USERNAME" && \
        useradd -m -u 1000 -g "$USERNAME" -s /bin/bash "$USERNAME"; \
    fi && \
    usermod -aG sudo "$USERNAME" && \
    printf '%s ALL=(ALL) NOPASSWD:ALL\n' "$USERNAME" > /etc/sudoers.d/flatland && \
    chmod 0440 /etc/sudoers.d/flatland && \
    mkdir -p /opt/overlay_ws/src /tmp/.ccache && \
    chown -R "$USERNAME:$USERNAME" /opt/overlay_ws /tmp/.ccache
USER ${USERNAME}