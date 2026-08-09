cmake_minimum_required(VERSION 3.8)
project(fast_livo)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-pthread -fexceptions)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  add_compile_options($<$<CONFIG:Release>:-O3> $<$<CONFIG:Release>:-mcpu=native>
                      $<$<CONFIG:Release>:-mtune=native> $<$<CONFIG:Release>:-ffast-math>)
  add_compile_definitions(ARM_ARCH)
else()
  add_compile_options($<$<CONFIG:Release>:-O3> $<$<CONFIG:Release>:-march=native>
                      $<$<CONFIG:Release>:-mtune=native> $<$<CONFIG:Release>:-funroll-loops>)
  add_compile_definitions(X86_ARCH)
endif()

add_compile_definitions(ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")
include(ProcessorCount)
ProcessorCount(PROCESSOR_COUNT)
if(PROCESSOR_COUNT GREATER 4)
  add_compile_definitions(MP_EN MP_PROC_NUM=4)
elseif(PROCESSOR_COUNT GREATER 1)
  add_compile_definitions(MP_EN MP_PROC_NUM=${PROCESSOR_COUNT})
else()
  add_compile_definitions(MP_PROC_NUM=1)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenMP QUIET)
find_package(mimalloc QUIET)

set(ROS_DEPENDENCIES
  rclcpp sensor_msgs nav_msgs geometry_msgs visualization_msgs
  pcl_conversions cv_bridge image_transport tf2 tf2_ros)

add_library(vio src/vio.cpp src/frame.cpp src/visual_point.cpp)
add_library(lio src/voxel_map.cpp)
add_library(pre src/preprocess.cpp)
add_library(imu_proc src/IMU_Processing.cpp)
add_library(laser_mapping src/LIVMapper.cpp)

foreach(target vio lio pre imu_proc laser_mapping)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
  ament_target_dependencies(${target} ${ROS_DEPENDENCIES})
endforeach()

target_link_libraries(vio ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(lio ${PCL_LIBRARIES})
target_link_libraries(pre ${PCL_LIBRARIES})
target_link_libraries(imu_proc ${PCL_LIBRARIES})
target_link_libraries(laser_mapping vio lio pre imu_proc ${PCL_LIBRARIES}
                      ${OpenCV_LIBRARIES} Boost::thread)

add_executable(fastlivo_mapping src/main.cpp)
target_include_directories(fastlivo_mapping PRIVATE include ${EIGEN3_INCLUDE_DIRS}
                           ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(fastlivo_mapping ${ROS_DEPENDENCIES})
target_link_libraries(fastlivo_mapping laser_mapping)

if(OpenMP_CXX_FOUND)
  foreach(target vio lio pre imu_proc laser_mapping fastlivo_mapping)
    target_link_libraries(${target} OpenMP::OpenMP_CXX)
  endforeach()
endif()
if(mimalloc_FOUND)
  target_link_libraries(fastlivo_mapping mimalloc)
endif()

install(TARGETS fastlivo_mapping DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz_cfg DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/play_all_bags.sh scripts/generate_mine_calibration.py
        scripts/prepare_wuhu_bag.py scripts/evaluate_wuhu_lio.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
