
cmake_minimum_required(VERSION 3.5)
project(orbslam)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")
set(HOME /home/frank20a)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

add_compile_options(-Wno-all -Wno-reorder -Wno-deprecated-declarations -Wno-pedantic)


set(ORB_SLAM3_DIR ${HOME}/dev-ws/ORB_SLAM3)

set(CMAKE_PREFIX_PATH ${ORB_SLAM3_DIR}:${CMAKE_PREFIX_PATH})

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_viewer g2o_ext_csparse g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse)

find_package(OpenCV 4.6 EXACT REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
  # ${ORB_SLAM3_DIR}/Thirdparty/DBoW2
  # ${ORB_SLAM3_DIR}/Thirdparty/g2o
  ${Pangolin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(${ORB_SLAM3_DIR}/lib/)
link_directories(${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/)

set(LIBS 
  ${OpenCV_LIBS} 
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
  /opt/ros/galactic/lib/libcv_bridge.so
  -lboost_system
)

# Monocular
add_executable(mono
  src/test/ros_mono.cc
)

target_link_libraries(mono
  ${LIBS}
)

ament_target_dependencies(
  mono 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  tf2_ros 
  cv_bridge 
  message_filters 
  Pangolin 
  OpenCV
)

# Monocular Inertial
add_executable(mono_inertial
  src/test/ros_mono_inertial.cc
)

target_link_libraries(mono_inertial
  ${LIBS}
)

ament_target_dependencies(
  mono_inertial 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  tf2_ros 
  cv_bridge 
  message_filters 
  Pangolin 
  OpenCV
)

# Create targets
install(TARGETS mono mono_inertial
  DESTINATION lib/${PROJECT_NAME})

ament_package()
