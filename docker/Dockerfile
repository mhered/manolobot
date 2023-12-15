# base image on ros:humble
FROM osrf/ros:humble-desktop-full

# Install nano file editor and remove apt cache to keep image size down   
RUN apt-get update \
    && apt-get install -y \ 
    nano \ 
    && rm -rf /var/lib/apt/lists/*

# Copy the config folder to the robot_config folder
COPY /config /robot_config/

# Define arguments for the non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user with the same user id as the host user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \   
    && mkdir /home/$USERNAME/.config \ 
    && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo: install sudo, add user to sudo group, allow sudo without password
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Install apps for testing various devices
RUN apt-get update \
    && apt-get install -y\
    evtest \
    jstest-gtk \
    python3-serial \
    usbutils \ 
    guvcview \
    dbus-x11 \
    && rm -rf /var/lib/apt/lists/*

# Set the display environment variable
ENV DISPLAY=:0

# Add user to video group
RUN usermod -aG video $USERNAME 

# Copy entrypoint script into image
COPY entrypoint.sh /entrypoint.sh

# Copy bashrc script into image
COPY bashrc /home/${USERNAME}/.bashrc

# Execute it
ENTRYPOINT [ "/bin/bash", "/entrypoint.sh" ]

# Set the default command to bash to have a working terminal
CMD ["bash"]




# not needed if base is official ROS distro
# #install language support
# RUN apt-get update && apt-get install -y \
#     locales \
#     && locale-gen en_US.UTF-8 \
#     && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
#     && rm -rf /var/lib/apt/lists/* \
# ENV LANG en_US.UTF-8

# # install time zone support
# RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
#     && export DEBIAN_FRONTEND=noninteractive \
#     && apt-get update \
#     && apt-get install -y tzdata \
#     && dpkg-reconfigure --frontend noninteractive tzdata \
#     && rm -rf /var/lib/apt/lists/*

# autocompletion for bash and python
RUN apt-get update && apt-get install -y \
bash-completion \
python3-argcomplete \
&& rm -rf /var/lib/apt/lists/*