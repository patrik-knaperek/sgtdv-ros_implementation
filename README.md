# ROS_implementation

Implementation for ros in c++

#Pre pouzitie docker
Momentalne je spravena iba konzolova verzia ubuntu 18.04 s povolenimi oknami (cize zobrazenie aplikacnych okien funguje)  

Treba mat nainstalovany nvidia container https://github.com/NVIDIA/nvidia-docker  
Konkretne:  
### Ubuntu 16.04/18.04/20.04, Debian Jessie/Stretch/Buster
```sh
# Add the package repositories
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

#### CentOS 7.X/8.X (docker-ce), RHEL 7.X/8.X (docker-ce), Amazon Linux 1/2
```
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.repo | sudo tee /etc/yum.repos.d/nvidia-docker.repo

sudo yum install -y nvidia-container-toolkit
sudo systemctl restart docker
```

Spustit cez  prikaz (treba zmenit cesu k suborom v ```-v``` casti)
```sh
docker build -f Dockerfile-gl -t sgt_ros_implementation_gl . && docker run -p 5900:5901 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v <your path to ros_implementation>:/usr/src/ROS_implementation/ --privileged -it --network host --gpus all sgt_ros_implementation_gl
```

A nasledne skompilovanie:
```sh
cd /usr/src/ROS_implementation/  
catkin_make -j1 -l1
```

