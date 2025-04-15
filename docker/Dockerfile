FROM ros:noetic-perception-focal

# Set the environment variable to non-interactive
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary packages
RUN apt-get update && \
    apt-get install -y build-essential nano \
    ros-noetic-desktop-full net-tools \
    tmux \
    iputils-ping && \
    rm -rf /var/lib/apt/lists/*
    
#forgot a few stuff    
RUN apt-get update && \
    apt-get install -y net-tools iproute2 ros-noetic-plotjuggler* ros-noetic-foxglove-bridge ros-noetic-turtlesim* gedit    

# Create a non-root user named 'robotics' with home directory
RUN useradd -m -s /bin/bash robotics && \
    mkdir -p /home/robotics/data && \
    mkdir -p /home/robotics/catkin_ws/src && \
    chown -R robotics:robotics /home/robotics

# Switch to the robotics user
USER robotics
WORKDIR /home/robotics

# Source ROS environment for the user
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
    echo "source /home/robotics/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Default command
CMD ["bash"]

