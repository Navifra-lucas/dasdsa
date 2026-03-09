# 프로젝트 설명
Navifra brain ros agent

## Prerequisite
 - Poco 1.11.6
 - roscpp
 - roslib
 - core_msgs
 - core
 - tf2_ros
 - tf
 - yaml-cpp

## Poco Installl 
### Poco는  Static Version를 설차히세요.
```bash
mkdir ~/install && cd ~/install
wget https://github.com/pocoproject/poco/archive/refs/tags/poco-1.11.6-release.zip 
unzip /root/install/poco-1.11.6-release.zip -d ~/install
cd ~/install/poco-poco-1.11.6-release && ./build_cmake.sh -DBUILD_SHARED_LIBS=OFF
```

## BUILD
```bash
catkin_make -DBUILD_TYPE=Release
```
