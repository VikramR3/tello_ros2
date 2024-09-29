# To build:
# docker build --pull --no-cache --tag tello_ros:foxy .

# To run:
# docker run -it tello_ros:foxy bash

# I'm using this for smoke tests
# To run tello_ros in a docker container you will need to set up ports, x-windows, etc.

FROM osrf/ros:foxy-desktop

RUN apt-get update
RUN apt-get upgrade -y

RUN apt-get install -y libasio-dev
RUN apt-get install -y python3-pip
RUN yes | pip3 install 'transformations==2018.9.5'
RUN apt-get update && apt-get install -y curl && \
    curl -o requirements.txt https://raw.githubusercontent.com/VikramR3/tello_ros2/main/requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

WORKDIR /work/tello_ros2_ws/src

RUN git clone https://github.com/VikramR3/tello_ros2.git
RUN git clone https://github.com/ptrmu/ros2_shared.git
RUN git clone https://github.com/ptrmu/fiducial_vlam.git

WORKDIR /work/tello_ros2_ws

RUN rosdep install -y --from-paths . --ignore-src

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build"

RUN echo 'source /work/tello_ros_ws/install/setup.bash' >> /root/.bashrc

