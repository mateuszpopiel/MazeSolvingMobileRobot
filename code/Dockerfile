FROM osrf/ros:humble-desktop-full

RUN mkdir -p /home/ws/src
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
WORKDIR /home/ws

# install an img with a Gazebo
RUN apt-get update && apt-get upgrade -y
RUN apt install lsb-release wget gnupg -y
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get upgrade -y
RUN apt-get install gz-garden -y

ENTRYPOINT ["/bin/bash", "-c"]
