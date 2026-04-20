FROM fishros2/ros:noetic-desktop-full

# Fix GPG Keys
RUN apt-get update || true && \
    apt-get install -y curl gnupg2 && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Upgrade and Install Dependencies
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y python3-pip python3-tk python3-matplotlib \
    ros-noetic-moveit ros-noetic-moveit-core ros-noetic-moveit-ros-planning ros-noetic-moveit-ros-planning-interface \
    ros-noetic-kdl-parser-py ros-noetic-urdf-parser-plugin python3-pykdl \
    ros-noetic-ros-control ros-noetic-ros-controllers \
    ros-noetic-effort-controllers ros-noetic-position-controllers ros-noetic-joint-state-controller && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /root/ur5_gazebo

# Install Python Libraries from the shared requirements file.
COPY requirements.txt /root/ur5_gazebo/requirements.txt
RUN python3 -m pip install -r requirements.txt --no-cache-dir

# Copy project files
COPY . /root/ur5_gazebo/
RUN chmod +x /root/ur5_gazebo/*.sh

# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

ENTRYPOINT ["/root/ur5_gazebo/docker_entrypoint.sh"]
