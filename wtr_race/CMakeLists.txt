cmake_minimum_required(VERSION 2.8.3)

project(wtr_race)

# set(CMAKE_CXX_FLAGS "-std=c++11")
# set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

find_package(PythonLibs REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(fmt)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  visualization_msgs
  geometry_msgs
  std_msgs
  tf
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
  nav_core 
  pluginlib
  cv_bridge
  image_transport
  sensor_msgs
  pcl_ros
  message_filters
)

include_directories(
  SYSTEM
  include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/wtr_race/
    ${CMAKE_CURRENT_SOURCE_DIR}/include/astar/
    ${CMAKE_CURRENT_SOURCE_DIR}/include/livox_lidar/
    ${CMAKE_CURRENT_SOURCE_DIR}/include/wtr_mavros_msgs/
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}    mr_planner_lib
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ../include
    
    
    ../../../devel/include
    /usr/include/x86_64-linux-gnu/qt5
    /usr/include/x86_64-linux-gnu/qt5/QtGui
    /usr/include/x86_64-linux-gnu/qt5/QtCore
    ${PYTHON_INCLUDE_DIRS}
)

set(THIRD_PARTY
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    fmt::fmt
    )

add_message_files(
  FILES
    wtr_posture.msg
    wtr_zone.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_receiver
 CATKIN_DEPENDS roscpp std_msgs message_runtime nav_core pluginlib 
#  DEPENDS system_lib
)

add_library(Astar_planner_lib
  include/astar/astar.h
  src/astar/astar.cpp
)

add_executable(traj_plan_3D src/traj_plan_3D.cpp)
# add_executable(loop_dect  src/visual_dect/loop_dect.cpp)
# add_executable(decaymap_detect  src/pointcloud/decaymap_detect.cpp)

# add_dependencies(traj_plan_3D ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(traj_plan_3D ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(Astar_planner_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(Decaymap_detect_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(loop_dect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(decaymap_detect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

link_directories(
  ${catkin_LIB_DIRS}
  ${PCL_LIBRARY_DIRS}
)
target_link_libraries(Astar_planner_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(traj_plan_3D
  Astar_planner_lib
  # Decaymap_detect_lib
  ${catkin_LIBRARIES}
)
# target_link_libraries(loop_dect
#   ${catkin_LIBRARIES}
#   # ${OpenCV_LIBS}
#   ${OpenCV_LIBRARIES}
# )
# target_link_libraries(decaymap_detect
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${THIRD_PARTY}
# )

# catkin_install_python(PROGRAMS scripts/ros/video_tackle.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
 

