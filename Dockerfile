FROM osrf/ros:humble-desktop

#install tools, turtleBot3 and cam drivers
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3* \
    ros-humble-usb-cam \
    ros-humble-gazebo-* \
    python3-pip \
    python3-opencv \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

#set up the turtleBot3 model (waffle has a camera)
ENV TURTLEBOT3_MODEL=waffle
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

#automatically source ROS when you open the terminal
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
