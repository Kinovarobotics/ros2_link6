ARG ROS_DISTRO=humble

################################
#        Base ROS Image        #
################################
FROM ros:${ROS_DISTRO} AS base

ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV RCUTILS_COLORIZED_OUTPUT=1

# Set the shell to support the "source" command
SHELL ["/bin/bash", "-eo", "pipefail", "-c"]

# Install basic apt packages
RUN apt-get update && apt-get install -y --no-install-recommends \
 wget git python3 python3-pip python3-vcstool python3-colcon-common-extensions \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

# Use Cyclone DDS as middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# The Kortex API bindings require exactly Protobuf 3.20.3
RUN cd /tmp \
  && wget https://github.com/protocolbuffers/protobuf/releases/download/v3.20.3/protobuf-cpp-3.20.3.tar.gz \
  && tar -xzf protobuf-cpp-3.20.3.tar.gz \
  && cd protobuf-3.20.3 \
  && ./configure --prefix=/usr/local \
  && make -j$(nproc) \
  && make install \
  && ldconfig

# Create a base Colcon workspace with external dependencies
RUN mkdir -p /base_ws/src
WORKDIR /base_ws/src
COPY ros2_link6.${ROS_DISTRO}.repos .
RUN vcs import < ros2_link6.${ROS_DISTRO}.repos

# Build the base Colcon workspace, installing dependencies first.
WORKDIR /base_ws
RUN apt-get update -y \
 && rosdep update \
 && source /opt/ros/${ROS_DISTRO}/setup.bash \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -r -y \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/* \
 && colcon build --packages-skip cartesian_controller_simulation cartesian_controller_tests


###########################################
# Deployment Image with the projec's code #
###########################################
FROM base AS deploy

# Create an overlay Colcon workspace and copy the project source code
RUN mkdir -p /overlay_ws
WORKDIR /overlay_ws
COPY . /overlay_ws/

# Build workspace, installing dependencies first.
RUN apt-get update -y \
 && source /opt/ros/${ROS_DISTRO}/setup.bash \
 && source /base_ws/install/setup.bash \
 && rosdep install --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} -r -y \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/* \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Set up the entrypoint
WORKDIR /overlay_ws
COPY ./.devcontainer/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]


#####################
# Development Image #
#####################
FROM deploy AS dev

# Dev container arguments
ARG USERNAME=devuser
ARG USER_UID=1001
ARG USER_GID=${USER_UID}

# Install extra tools for development
RUN apt-get update && apt-get install -y --no-install-recommends nano net-tools iputils-ping \
    ros-${ROS_DISTRO}-rqt* ros-${ROS_DISTRO}-teleop-twist-keyboard gdbserver gdb \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u ${USER_UID} ; then userdel `id -un ${USER_UID}` ; fi

# Create new user and home directory
RUN groupadd --gid ${USER_GID} ${USERNAME} \
 && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

# Set the ownership of the base workspace to the new user
RUN chown -R ${USER_UID}:${USER_GID} /overlay_ws/

# Set up the entrypoint, including it in the .bashrc for interactive shells
USER ${USERNAME}
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc