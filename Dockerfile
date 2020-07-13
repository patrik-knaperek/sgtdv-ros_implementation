# https://github.com/stereolabs/zed-docker/tree/master/3.X/ubuntu
FROM stereolabs/zed:3.2-ros-devel-cuda10.0-ubuntu18.04

# set work directory
WORKDIR /usr/src/ROS_implementation

# Make darknet dir
RUN mkdir darknet
# download darkent from github
RUN git clone https://github.com/AlexeyAB/darknet.git /usr/src/ROS_implementation/darknet

# Build lib
# RUN cd /usr/src/ROS_implementation/darknet && sed -i 's/GPU=0/GPU=1/g' Makefile && sed -i 's/CUDNN=0/CUDNN=1/g' Makefile && sed -i 's/CUDNN_HALF=0/CUDNN_HALF=1/g' Makefile && sed -i 's/OPENCV=0/OPENCV=1/g' Makefile && sed -i 's/ZED_CAMERA=0/ZED_CAMERA=1/g' Makefile && sed -i 's/LIBSO=0/LIBSO=1/g' Makefile && make

# Build lib without ZED SDK and without CUDNN_HALF
RUN cd /usr/src/ROS_implementation/darknet && sed -i 's/GPU=0/GPU=1/g' Makefile && sed -i 's/CUDNN=0/CUDNN=1/g' Makefile && sed -i 's/OPENCV=0/OPENCV=1/g' Makefile && sed -i 's/LIBSO=0/LIBSO=1/g' Makefile && make

RUN cd /usr/src/ROS_implementation/

# update and upgrade ubuntu and install vim editor
RUN apt-get update -y && apt-get upgrade -y && apt-get install vim -y

# copy project
COPY . .

#edit dependencies as cuda and darknet
RUN sed -i 's|  /usr/local/cuda-10.0/include # include cuda 10.0|  /usr/local/cuda-10.0/include # include cuda 10.0|g' /usr/src/ROS_implementation/src/camera_cone_detection/CMakeLists.txt
RUN sed -i 's|    /usr/src/ROS_implementation/darknet/libdarknet.so #treba zmenit na aktualnu cestu|    /usr/src/ROS_implementation/darknet/libdarknet.so #treba zmenit na aktualnu cestu|g' /usr/src/ROS_implementation/src/camera_cone_detection/CMakeLists.txt

RUN cd /usr/src/ROS_implementation/
# padne docker nwm preco
#CMD ["catkin_make", "-j1", "-l1"]

#RUN cp /usr/src/ROS_implementation/yolov3-tiny_3l.weights /usr/src/ROS_implementation/devel/lib/camera_cone_detection/yolov3-tiny.weights
#RUN cp /usr/src/ROS_implementation/yolov3-tiny_3l.cfg /usr/src/ROS_implementation/devel/lib/camera_cone_detection/yolov3-tiny.cfg
#RUN cp /usr/src/ROS_implementation/druha_jazda.svo /usr/src/ROS_implementation/devel/lib/camera_cone_detection/druha_jazda.svo

### TRY PCL
