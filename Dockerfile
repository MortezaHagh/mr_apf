# FROM ros:noetic-desktop-full
ARG BASE_IMAGE=osrf/ros:noetic-desktop-full
FROM ${BASE_IMAGE}

ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

HEALTHCHECK NONE

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if [ -n "$USER_UID" ] && id -u "$USER_UID" >/dev/null 2>&1; then \
      userdel "$(id -un "$USER_UID")"; \
    fi

## Update package lists and install required packages
RUN apt-get update \
  && apt-get install -y --no-install-recommends ca-certificates git gnupg lsb-release curl sudo zsh rsync python3-pip \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# Create a non-root user to use if preferred
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
  #
  # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

########################################################################################################################
##  Environment, tools and CI 
RUN apt-get update && apt-get install -y --no-install-recommends python3-autopep8 python3-tk \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

########################################################################################################################
##  Setting up bashrc and zshrc 
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/$USERNAME/.bashrc \
  && echo "source /home/dev_ws/devel/setup.bash" >> /home/$USERNAME/.bashrc

USER $USERNAME

RUN RUNZSH=no CHSH=no KEEP_ZSHRC=yes sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" && \
  echo "source /opt/ros/noetic/setup.zsh" >> /home/$USERNAME/.zshrc && \
  echo "source /home/dev_ws/devel/setup.zsh" >> /home/$USERNAME/.zshrc && \
  echo "" >> /home/$USERNAME/.zshrc && \
  echo "export DISABLE_ROS1_EOL_WARNINGS=true" >> /home/$USERNAME/.zshrc && \
  echo "export DISABLE_ROS1_EOL_WARNINGS=true" >> /home/$USERNAME/.bashrc

########################################################################################################################
# ROS packages
# switch to root user to install system packages
USER root

# Add ROS apt repository and key
RUN apt-get update \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
 && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends ros-noetic-desktop-full ros-noetic-turtlebot3 sudo \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*


RUN apt-get update \
  && apt-get upgrade -y \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# Install Python dependencies (upgrade pip/tools, avoid caching)
RUN python3 -m pip install numpy matplotlib plotly shapely colorama pandas

########################################################################################################################
#  Default Shell 

# ENV SHELL /bin/bash
# USER $USERNAME
# CMD ["/bin/bash"]

ENV SHELL /usr/bin/zsh
USER $USERNAME
CMD ["/usr/bin/zsh"]
