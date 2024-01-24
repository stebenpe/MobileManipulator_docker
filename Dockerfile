FROM nvcr.io/nvidia/l4t-ml:r36.2.0-py3

#user arguments
ARG USERNAME=moma
ARG USER_UID=1000
ARG USER_GID=$USER_UID

#create non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config &&chown $USER_UID:$USER_GID /home/$USERNAME/.config

#set up sudo
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/list/*

USER root

RUN apt-get update
RUN apt-get upgrade -y

#install tools
RUN apt-get install -y \
    nano \
    git

##ros2
#remove opencv becouse of broken pipe
RUN apt-get purge -y '*opencv*'

#enable universe repository
RUN apt-get install software-properties-common -y
RUN add-apt-repository universe -y

#add the ROS 2 GPG key with apt
RUN apt-get update && apt-get install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

#add repository to sources list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN rm -rf /var/lib/apt/list/*
RUN apt-get update
RUN apt-get upgrade -y

#install ros2 humble desktop
RUN apt-get install ros-humble-desktop -y

#install compiler and other tools
RUN apt-get install ros-dev-tools -y

#install ros2 dependencies
RUN apt-get install -y \
    ros-humble-tf2* \
    ros-humble-control-msgs

# install ros2 pip library's
RUN apt-get install pip -y
RUN pip install \
    pymodbus \
    selectors2 \
    pyserial

##website server
#npm
RUN apt-get install npm -y

#nvm
USER $USERNAME
RUN ["/bin/bash", "-c", "curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash"]
# RUN echo -e 'export NVM_DIR="$HOME/.nvm"\n[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm\n[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"  # This loads nvm bash_completion' >> /home/moma/.bashrc
# RUN ["/bin/bash", "-c", "source ~/.bashrc"]
# RUN ["/bin/bash", "-c", "nvm install 18.12.0"]
# RUN ["/bin/bash", "-c", "nvm use 18.12.0"]

USER root
#ros2
RUN apt install ros-humble-rosbridge-server -y

#USER $USERNAME

COPY MobileManipulator_website/vue-webpanel /home/$USERNAME/vue-webpanel
WORKDIR /home/$USERNAME/vue-webpanel
RUN npm install

COPY MobileManipulator_ros2/ros2_ws /home/$USERNAME/ros2_ws

COPY entrypoint.sh /entrypoint.sh

ENTRYPOINT [ "/bin/bash", "/entrypoint.sh" ]

CMD ["bash"]