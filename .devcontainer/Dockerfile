ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    ccache \
    gdb \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

COPY . /tmp/flatland
RUN rosdep update && \
    apt-get update && \
    rosdep install --from-paths /tmp/flatland --ignore-src --rosdistro "$ROS_DISTRO" -y && \
    rm -rf /tmp/flatland /var/lib/apt/lists/*

ENV CCACHE_DIR=/tmp/.ccache
WORKDIR /opt/overlay_ws