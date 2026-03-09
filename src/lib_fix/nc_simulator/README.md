# ROS with PyQT5
_QT GUI communicating with ROS_

## Install

    sudo apt-get install qtcreator
    sudo apt-get install build-essential libgl1-mesa-dev
    sudo apt-get install qt5-default
    sudo apt-get install build-essential make


## How to run
First, give permission to execute python scripts.

    cd ~/{catkin_workspace}/src/3_core_sensor/nc_simulator/scripts/
    chmod +x virtual_obs_controller.py

Then in your command prompt,

`
roslaunch nc_simulator virtual_obs_controller.launch
`

## Result Image
![image](https://github.com/navifra/3_core_sensor/assets/41279501/74e3afac-0507-462d-83ad-265726969d3e)

### 1. To update your newly made obstacle
   
![image](https://github.com/navifra/3_core_sensor/assets/41279501/fc4c640b-698c-4892-b86c-698e5fee1db0)

Edit position and size. Then push **ok** button to confirm.
This button only works when publishing state is **False**

### 2. To start publishing
   
![image](https://github.com/navifra/3_core_sensor/assets/41279501/cddea1e6-1cd3-4559-b8f7-2df96ac1ee1f)

Toggle on/off button. Default mode is off(=publisher off)

### 3. While publishing

You can change linear and angular velocity with sliders!
