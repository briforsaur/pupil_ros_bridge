cmake_minimum_required(VERSION 3.0.2)
project(pupil_ros_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  Cartesian2D.msg
  Cartesian3D.msg
  Circle3D.msg
  Ellipse.msg
  EllipseAxes.msg
  Gaze.msg
  ProjectedSphere.msg
  Pupil.msg
  Sphere.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS rospy message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/pupil_subscriber.py
  scripts/gaze_subscriber.py
  scripts/pupil_ipc_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
