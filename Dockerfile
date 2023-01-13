FROM cyberbotics/webots
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
RUN apt update && apt install -y curl gnupg2 lsb-release git-all
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update
RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-foxy-desktop
WORKDIR /root/dev_ws/src
RUN git clone https://github.com/ros/ros_tutorials.git -b foxy-devel
WORKDIR /root/dev_ws
RUN apt-get install python3-rosdep -y
RUN rosdep init
RUN rosdep update
RUN rosdep install -i --from-path src --rosdistro foxy -y
RUN apt install python3-colcon-common-extensions -y
COPY ros2_entrypoint.sh /root/.
COPY scripts /home/.
ENTRYPOINT ["/root/ros2_entrypoint.sh"]
CMD ["bash"]
#docker image tag b094d548233b webotsros2:latest
# xvfb-run webots --stdout --stderr --batch --mode=realtime /path/to/your/world/file

# docker build -t ros2wscript . 