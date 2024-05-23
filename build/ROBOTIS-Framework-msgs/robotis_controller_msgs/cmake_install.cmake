# Install script for directory: /home/hsh/catkin_ws/src/ROBOTIS-Framework-msgs/robotis_controller_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/hsh/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotis_controller_msgs/msg" TYPE FILE FILES
    "/home/hsh/catkin_ws/src/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/SyncWriteItem.msg"
    "/home/hsh/catkin_ws/src/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/JointCtrlModule.msg"
    "/home/hsh/catkin_ws/src/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/StatusMsg.msg"
    "/home/hsh/catkin_ws/src/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg/WriteControlTable.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotis_controller_msgs/srv" TYPE FILE FILES
    "/home/hsh/catkin_ws/src/ROBOTIS-Framework-msgs/robotis_controller_msgs/srv/GetJointModule.srv"
    "/home/hsh/catkin_ws/src/ROBOTIS-Framework-msgs/robotis_controller_msgs/srv/SetJointModule.srv"
    "/home/hsh/catkin_ws/src/ROBOTIS-Framework-msgs/robotis_controller_msgs/srv/SetModule.srv"
    "/home/hsh/catkin_ws/src/ROBOTIS-Framework-msgs/robotis_controller_msgs/srv/LoadOffset.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotis_controller_msgs/cmake" TYPE FILE FILES "/home/hsh/catkin_ws/build/ROBOTIS-Framework-msgs/robotis_controller_msgs/catkin_generated/installspace/robotis_controller_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/hsh/catkin_ws/devel/include/robotis_controller_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/hsh/catkin_ws/devel/share/roseus/ros/robotis_controller_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/hsh/catkin_ws/devel/share/common-lisp/ros/robotis_controller_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/hsh/catkin_ws/devel/share/gennodejs/ros/robotis_controller_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/hsh/catkin_ws/devel/lib/python3/dist-packages/robotis_controller_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/hsh/catkin_ws/devel/lib/python3/dist-packages/robotis_controller_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/hsh/catkin_ws/build/ROBOTIS-Framework-msgs/robotis_controller_msgs/catkin_generated/installspace/robotis_controller_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotis_controller_msgs/cmake" TYPE FILE FILES "/home/hsh/catkin_ws/build/ROBOTIS-Framework-msgs/robotis_controller_msgs/catkin_generated/installspace/robotis_controller_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotis_controller_msgs/cmake" TYPE FILE FILES
    "/home/hsh/catkin_ws/build/ROBOTIS-Framework-msgs/robotis_controller_msgs/catkin_generated/installspace/robotis_controller_msgsConfig.cmake"
    "/home/hsh/catkin_ws/build/ROBOTIS-Framework-msgs/robotis_controller_msgs/catkin_generated/installspace/robotis_controller_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotis_controller_msgs" TYPE FILE FILES "/home/hsh/catkin_ws/src/ROBOTIS-Framework-msgs/robotis_controller_msgs/package.xml")
endif()

