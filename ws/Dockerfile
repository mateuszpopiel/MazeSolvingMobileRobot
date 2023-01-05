# Build a docker image with `docker build -t osrf/ros:humble-gazeebo .`
# 
FROM osrf/ros:humble-desktop-full

RUN mkdir -p /home/ws/src
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
WORKDIR /home/ws

USER root
  RUN useradd -m developer && \
    echo "developer:123" | chpasswd && \
    adduser developer sudo
  RUN apt-get update && apt-get upgrade -y
  RUN apt install lsb-release wget gnupg -y
  RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
  RUN apt-get update && apt-get upgrade -y
  RUN apt-get install gz-garden -y

USER developer
  RUN mkdir -p /home/developer/ws/src
  RUN echo "source /opt/ros/humble/setup.bash" >> /home/developer/.bashrc
  WORKDIR /home/developer/ws
  RUN /bin/bash -c \
    '. /opt/ros/humble/setup.bash; \
    cd /home/developer/ws; \
    colcon build'
  RUN echo "source /home/developer/ws/devel/setup.bash" >> /home/developer/.bashrc
  RUN echo "export ROS_MASTER_URI=http://Vostro5481:11311" >> /home/developer/.bashrc

ENTRYPOINT ["/bin/bash", "-c"]
