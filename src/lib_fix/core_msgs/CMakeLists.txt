cmake_minimum_required(VERSION 3.0.2)
project(core_msgs)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    message_generation
)
find_package(Python3 REQUIRED)

add_message_files(
  FILES
  # navi
  Goal.msg
  GoalList.msg
  NavicoreStatus.msg
  NaviStatus.msg
  NaviAlarm.msg
  NaviAlarmRos2.msg
  MaxTwist.msg
  MotorInfo.msg
  Battery.msg
  BatteryInfo.msg
  BmsInfo.msg
  RedisMotor.msg
  PGVInfoHik.msg
  PGVInfo.msg
  PGVPose.msg
  PGVPoseList.msg
  PLCInfo.msg
  VisionObs.msg
  VisionObsList.msg
  LidarInfoMsg.msg
  LidarDistList.msg
  UpperInfo.msg
  CameraInfo.msg
  CameraInfoList.msg
  CameraRoiInfo.msg
  CameraRoiInfoList.msg
  CameraRoiInfoWia.msg
  UniconInfo.msg
  UniconVel.msg
  Nav350Info.msg
  MotorServoStatus.msg
  SimVel.msg
  Can.msg
  ARPose.msg
  ARPoseList.msg
  EachMotor.msg
  ORBDeviceInfo.msg
  ORBIMUInfo.msg

  HacsNode.msg
  HacsNodeList.msg
  
  # map
  MapDB.msg
  JsonLinkList.msg
  JsonList.msg
  JsonNodeList.msg
  MapJson.msg

  # brain
  RobotInfo.msg 
  AddTask.msg
  TaskAlarm.msg
  HacsNodeList.msg
  HacsNode.msg
  LaneAvoidMonit.msg
  TotalNodeList.msg
  TotalNode.msg
  
  # repeat test
  RepeatTestMsg.msg

  # localization
  LocalizeInfo.msg

  # calibration offset save
  CaliSave.msg

  # v2v
  Vehicle.msg
  VehicleList.msg

  #cheonil register
  CheonilReadRegister.msg

  #cheonil coil
  CheonilReadCoil.msg
  
  # robot pose
  RobotPose.msg
  Docking.msg
  
  # transform 
  TransformValue.msg

  DockInfo.msg

  # safety scanner
  SafetyScannerStatus.msg

  EtcInfo.msg
  
  #reflector 
  Reflector.msg
  
  #slam
  Graph.msg
  GraphNode.msg
  GraphLink.msg

  #nc_camera_merger
  IMUInfo.msg
  Extrinsics.msg
  Metadata.msg

  #fab
  Fab.msg
  Sound.msg
  
  #ultrasonic
  Ultrasonic.msg

  #motors
  MotorInfoRos2.msg
  MotorData.msg
  MotorDataInfo.msg

  #motion
  MotionInfo.msg

  #lidar
  LidarContamination.msg
    
  #mando
  LiftStatus.msg
  LiftInfo.msg
  LiftList.msg

  #led sound status
  LedSoundStatus.msg

  #hacs(wia)
  HacsWakeup.msg

  LiftHeightInfo.msg

  #Fork
  ForkLift.msg
  WiaForkInfo.msg

)

add_service_files(
  FILES
  #navicore
  CameraCmd.srv
  CameraRoi.srv
  DeviceInfo.srv
  CommonString.srv

  #orbbec
  GetBool.srv 
  GetCameraInfo.srv 
  GetCameraParams.srv 
  GetDeviceInfo.srv 
  GetInt32.srv 
  GetString.srv 
  SetBool.srv 
  SetInt32.srv 
  SetString.srv
)

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES 
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime message_generation geometry_msgs std_msgs nav_msgs
)

# YAML 관련 경로 설정
set(MSG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/msg/NaviAlarm.msg)
set(YAML_OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/msg/navi_alarm.yaml)
set(PYTHON_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/scripts/AlarmMsg2yaml.py)

# YAML 자동 생성 명령어 등록
add_custom_command(
    OUTPUT ${YAML_OUTPUT}
    COMMAND ${Python3_EXECUTABLE} ${PYTHON_SCRIPT}
            ${MSG_FILE}
            navi_alarm
            ${YAML_OUTPUT}
    DEPENDS ${PYTHON_SCRIPT} ${MSG_FILE}
    COMMENT "Generating YAML from NaviAlarm.msg"
    VERBATIM
)

# 항상 실행되는 타겟 등록
add_custom_target(generate_yaml ALL
    DEPENDS ${YAML_OUTPUT}
)

# 실제 라이브러리 타겟 정의 (예시)
add_library(${PROJECT_NAME}  msg/core_msg.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} generate_yaml)

# install 시 YAML도 포함
install(FILES ${YAML_OUTPUT}
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg)