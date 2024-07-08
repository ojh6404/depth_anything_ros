FROM nvcr.io/nvidia/tensorrt:23.03-py3
ENV DEBIAN_FRONTEND=noninteractive

# install essential packages
RUN apt update && apt install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    curl \
    wget \
    build-essential \
    git \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# setup keys
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

ENV ROS_DISTRO=noetic

# install ros core
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-core=1.5.0-1* \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-rosdep \
    python3-rosinstall \
    python3-osrf-pycommon \
    python3-catkin-tools \
    python3-wstool \
    python3-vcstools \
    python-is-python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# install ros packages
RUN apt update && apt install -y --no-install-recommends \
    ros-noetic-image-transport-plugins \
    ros-noetic-jsk-tools \
    ros-noetic-jsk-common \
    ros-noetic-jsk-topic-tools \
    && rm -rf /var/lib/apt/lists/*

# install point cloud library if needed
RUN apt-get update && apt-get install -y ros-noetic-jsk-pcl-ros ros-noetic-jsk-pcl-ros-utils &&\
    rm -rf /var/lib/apt/lists/*

########################################
########### WORKSPACE BUILD ############
########################################
# Installing catkin package
RUN pip install gdown
RUN /opt/tensorrt/install_opensource.sh
ENV TENSORRT_DIR=/workspace/tensorrt
ENV PATH=$TENSORRT_DIR/bin:$PATH
RUN mkdir -p ~/catkin_ws/src
RUN rosdep init && rosdep update && apt update
RUN git clone --recurse-submodules https://github.com/ojh6404/depth_anything_ros.git ~/catkin_ws/src/depth_anything_ros
RUN cd ~/catkin_ws/src/ &&\
    source /opt/ros/noetic/setup.bash &&\
    rosdep install --from-paths . -i -r -y &&\
    cd ~/catkin_ws && catkin init && catkin build
RUN pip install -r ~/catkin_ws/src/depth_anything_ros/requirements.txt --user &&\
    rm -rf ~/.cache/pip

# to avoid conflcit when mounting
RUN rm -rf ~/catkin_ws/src/depth_anything_ros/launch
RUN rm -rf ~/catkin_ws/src/depth_anything_ros/node_scripts

# ########################################
# ########### ENV VARIABLE STUFF #########
# ########################################
RUN touch ~/.bashrc
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

CMD ["bash"]
