###Setup base
#
#Base image can be tricky. In my oppinion you should only use a few base images. Complex ones with 
#everything usually have special use cases, an in my experience they take more time to understand, 
#than building one from the ground up.
#The base iamges I suggest you to use:
#- ubuntu: https://hub.docker.com/_/ubuntu
#- osrf/ros:version-desktop-full: https://hub.docker.com/r/osrf/ros
#- nvidia/cuda: https://hub.docker.com/r/nvidia/cuda
#
#We are mostly not space constrained so a little bigger image with everything is usually better,
#than a stripped down version.


FROM nvidia/cuda:11.3.1-cudnn8-devel-ubuntu20.04
#set shell 
SHELL ["/bin/bash", "-c"]
#set colors
ENV BUILDKIT_COLORS=run=green:warning=yellow:error=red:cancel=cyan
#start with root user
USER root

###Create new user
#
#Creating a user inside the container, so we won't work as root.
#Setting all setting all the groups and stuff.
#
###

#expect build-time argument
ARG HOST_USER_GROUP_ARG
#create group appuser with id 999
#create group hostgroup with ID from host. This is needed so appuser can manipulate the host files without sudo.
#create appuser user with id 999 with home; bash as shell; and in the appuser group
#change password of appuser to admin so that we can sudo inside the container
#add appuser to sudo, hostgroup and all default groups
#copy default bashrc and add ROS sourcing
RUN groupadd -g 999 appuser && \
    groupadd -g $HOST_USER_GROUP_ARG hostgroup && \
    useradd --create-home --shell /bin/bash -u 999 -g appuser appuser && \
    echo 'appuser:admin' | chpasswd && \
    usermod -aG sudo,hostgroup,plugdev,video,adm,cdrom,dip,dialout appuser && \
    cp /etc/skel/.bashrc /home/appuser/  

###Install the project
#
#If you install multiple project, you should follow the same 
#footprint for each:
#- dependencies
#- pre install steps
#- install
#- post install steps
#
###

#basic dependencies for everything and ROS
USER root
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive\
    apt-get install -y\
    netbase\
    git\
    build-essential\    
    wget\
    curl\
    gdb\
    lsb-release\
    sudo

# python install
USER root
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive\
    apt-get install -y\
    python3 \
    python3-pip \
    python3-dev 

    
#install ROS http://wiki.ros.org/noetic/Installation/Ubuntu
RUN rm -rf /var/lib/apt/lists/*
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive\
    apt-get install -y\
    ros-noetic-desktop-full

#install ROS tools
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive\
    apt-get install -y\
    python3-rosdep\
    python3-rosinstall\
    python3-rosinstall-generator\
    python3-catkin-tools\
    python3-wstool \
    ros-noetic-octomap \
    ros-noetic-octomap-msgs \
    ros-noetic-octomap-ros \
    ros-noetic-octomap-rviz-plugins \
    ros-noetic-pcl-ros \
    libomp-dev \
    libyaml-cpp-dev


RUN rosdep init
RUN rosdep update
#ros sourcing
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/appuser/.bashrc

USER root
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive\
    apt-get install -y\
    coinor-cbc


# switching back to appuser, so tha container starts there
USER appuser
RUN pip install pulp cupy-cuda11x open3d

# Install pybind11
USER root
RUN apt-get update && \ 
    DEBIAN_FRONTEND=noninteractive \
    apt-get install -y \
    pybind11-dev

USER appuser

