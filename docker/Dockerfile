FROM althack/ros2:humble-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    vim \
    nano \
    tmux \
    tree \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/ros2_ws

RUN mkdir -p src

COPY challenges/00_Introduction/challenge_c00 src/challenge_c00
COPY challenges/00_Introduction/challenge_c01 src/challenge_c01
COPY challenges/00_Introduction/challenge_c02 src/challenge_c02
COPY challenges/00_Introduction/challenge_c03 src/challenge_c03
COPY challenges/00_Introduction/challenge_c04 src/challenge_c04
COPY challenges/00_Introduction/challenge_c05 src/challenge_c05
COPY challenges/00_Introduction/challenge_c06 src/challenge_c06
COPY challenges/00_Introduction/challenge_c07 src/challenge_c07
COPY challenges/00_Introduction/challenge_c08 src/challenge_c08
COPY challenges/00_Introduction/challenge_c09 src/challenge_c09
COPY challenges/00_Introduction/challenge_c10 src/challenge_c10

RUN rosdep init || true && \
    rosdep update && \
    /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y"

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build"

RUN rm -rf src build log

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "alias ll='ls -lah'" >> /root/.bashrc && \
    echo "alias cb='cd /root/ros2_ws && colcon build'" >> /root/.bashrc && \
    echo "alias cs='cd /root/ros2_ws/src'" >> /root/.bashrc && \
    echo "echo ''" >> /root/.bashrc && \
    echo "echo '🎯 Welcome to NodeArena - Introduction Module!'" >> /root/.bashrc && \
    echo "echo ''" >> /root/.bashrc && \
    echo "echo '📦 Available Challenges:'" >> /root/.bashrc && \
    echo "echo '  C00-C10: Introduction challenges'" >> /root/.bashrc && \
    echo "echo ''" >> /root/.bashrc && \
    echo "echo '🔧 Quick Commands:'" >> /root/.bashrc && \
    echo "echo '  ros2 pkg list | grep challenge  - List challenge packages'" >> /root/.bashrc && \
    echo "echo '  ros2 run <package> <node>       - Run a challenge node'" >> /root/.bashrc && \
    echo "echo ''" >> /root/.bashrc

WORKDIR /root/ros2_ws

CMD ["/bin/bash"]