cmake_minimum_required(VERSION 3.8)
project(fast_livo)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

include(CTest)

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
  # Keep Eigen's alignment ABI identical to the Ubuntu PCL binaries. Enabling
  # a wider native SIMD ABI in this translation unit can make Eigen-aligned
  # point containers incompatible with the prebuilt PCL libraries and has
  # caused shutdown-time deallocation faults on this host.
  add_compile_options($<$<CONFIG:Release>:-O3>
                      $<$<CONFIG:Release>:-mtune=native>)
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
find_package(diagnostic_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io registration kdtree)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
set(MY_LIVO_GTSAM_PREFIX
    "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/gtsam/install-system-eigen"
    CACHE PATH "GTSAM install prefix built with the project's system Eigen")
find_package(GTSAM 4.2.2 REQUIRED CONFIG
  HINTS "${MY_LIVO_GTSAM_PREFIX}/lib/cmake/GTSAM"
  NO_DEFAULT_PATH)
find_package(OpenMP QUIET)
find_package(mimalloc QUIET)
if(NOT TARGET JsonCpp::JsonCpp)
  find_package(jsoncpp CONFIG REQUIRED)
endif()

set(ROS_DEPENDENCIES
  rclcpp sensor_msgs nav_msgs geometry_msgs diagnostic_msgs visualization_msgs
  pcl_conversions cv_bridge image_transport tf2 tf2_ros)

add_library(vio src/vio.cpp src/frame.cpp src/visual_point.cpp)
add_library(lio src/voxel_map.cpp)
add_library(pre src/preprocess.cpp)
add_library(imu_proc src/IMU_Processing.cpp)
add_library(backend_core
  src/backend/correction_feasibility_monitor.cpp
  src/backend/elastic_segment_acceptance_monitor.cpp
  src/backend/global_pose_layer.cpp
  src/backend/keyframe_manager.cpp
  src/backend/low_frequency_correction_field.cpp
  src/backend/loop_candidate_detector.cpp
  src/backend/loop_registration.cpp
  src/backend/loop_verifier.cpp
  src/backend/optimized_global_map.cpp
  src/backend/pose_graph_optimizer.cpp
  src/backend/regularized_correction_field_4d.cpp
  src/backend/rigid_relocalization_monitor.cpp
  src/backend/rtk_fusion_diagnostics.cpp
  src/backend/rtk_observation_buffer.cpp
  src/backend/rtk_velocity_guard.cpp)
add_library(laser_mapping src/LIVMapper.cpp)

foreach(target vio lio pre imu_proc backend_core laser_mapping)
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
target_link_libraries(backend_core ${PCL_LIBRARIES} gtsam)
target_link_libraries(laser_mapping vio lio pre imu_proc backend_core ${PCL_LIBRARIES}
                      ${OpenCV_LIBRARIES} Boost::thread JsonCpp::JsonCpp)

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

if(BUILD_TESTING)
  add_executable(voxel_map_reset_test tests/voxel_map_reset_test.cpp)
  target_include_directories(voxel_map_reset_test PRIVATE
    include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
  target_link_libraries(voxel_map_reset_test lio ${PCL_LIBRARIES})
  add_test(NAME voxel_map_reset_test COMMAND voxel_map_reset_test)

  add_executable(keyframe_manager_test tests/keyframe_manager_test.cpp)
  target_include_directories(keyframe_manager_test PRIVATE
    include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
  target_link_libraries(keyframe_manager_test backend_core ${PCL_LIBRARIES})
  add_test(NAME keyframe_manager_test COMMAND keyframe_manager_test)

  add_executable(global_pose_layer_test tests/global_pose_layer_test.cpp)
  target_include_directories(global_pose_layer_test PRIVATE
    include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
  target_link_libraries(global_pose_layer_test backend_core ${PCL_LIBRARIES})
  add_test(NAME global_pose_layer_test COMMAND global_pose_layer_test)

  add_executable(elastic_segment_acceptance_monitor_test
    tests/elastic_segment_acceptance_monitor_test.cpp)
  target_include_directories(elastic_segment_acceptance_monitor_test PRIVATE
    include ${EIGEN3_INCLUDE_DIRS})
  target_link_libraries(elastic_segment_acceptance_monitor_test backend_core)
  add_test(NAME elastic_segment_acceptance_monitor_test
    COMMAND elastic_segment_acceptance_monitor_test)

  add_executable(correction_feasibility_monitor_test
    tests/correction_feasibility_monitor_test.cpp)
  target_include_directories(correction_feasibility_monitor_test PRIVATE
    include ${EIGEN3_INCLUDE_DIRS})
  target_link_libraries(correction_feasibility_monitor_test backend_core)
  add_test(NAME correction_feasibility_monitor_test
    COMMAND correction_feasibility_monitor_test)

  add_executable(low_frequency_correction_field_test
    tests/low_frequency_correction_field_test.cpp)
  target_include_directories(low_frequency_correction_field_test PRIVATE
    include ${EIGEN3_INCLUDE_DIRS})
  target_link_libraries(low_frequency_correction_field_test backend_core)
  add_test(NAME low_frequency_correction_field_test
    COMMAND low_frequency_correction_field_test)

  add_executable(regularized_correction_field_4d_test
    tests/regularized_correction_field_4d_test.cpp)
  target_include_directories(regularized_correction_field_4d_test PRIVATE
    include ${EIGEN3_INCLUDE_DIRS})
  target_link_libraries(regularized_correction_field_4d_test backend_core)
  add_test(NAME regularized_correction_field_4d_test
    COMMAND regularized_correction_field_4d_test)

  add_executable(pose_graph_optimizer_test
    tests/pose_graph_optimizer_test.cpp)
  target_include_directories(pose_graph_optimizer_test PRIVATE
    include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
  target_link_libraries(pose_graph_optimizer_test backend_core)
  add_test(NAME pose_graph_optimizer_test COMMAND pose_graph_optimizer_test)

  add_executable(loop_candidate_detector_test
    tests/loop_candidate_detector_test.cpp)
  target_include_directories(loop_candidate_detector_test PRIVATE
    include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
  target_link_libraries(loop_candidate_detector_test backend_core)
  add_test(NAME loop_candidate_detector_test
    COMMAND loop_candidate_detector_test)

  add_executable(loop_registration_test
    tests/loop_registration_test.cpp)
  target_include_directories(loop_registration_test PRIVATE
    include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
  target_link_libraries(loop_registration_test backend_core)
  add_test(NAME loop_registration_test COMMAND loop_registration_test)

  add_executable(loop_verifier_test tests/loop_verifier_test.cpp)
  target_include_directories(loop_verifier_test PRIVATE
    include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
  target_link_libraries(loop_verifier_test backend_core)
  add_test(NAME loop_verifier_test COMMAND loop_verifier_test)

  add_executable(optimized_global_map_test
    tests/optimized_global_map_test.cpp)
  target_include_directories(optimized_global_map_test PRIVATE
    include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
  target_link_libraries(optimized_global_map_test backend_core ${PCL_LIBRARIES})
  add_test(NAME optimized_global_map_test COMMAND optimized_global_map_test)

  add_executable(rtk_observation_buffer_test
    tests/rtk_observation_buffer_test.cpp)
  target_include_directories(rtk_observation_buffer_test PRIVATE
    include ${EIGEN3_INCLUDE_DIRS})
  ament_target_dependencies(rtk_observation_buffer_test diagnostic_msgs)
  target_link_libraries(rtk_observation_buffer_test backend_core)
  add_test(NAME rtk_observation_buffer_test COMMAND rtk_observation_buffer_test)

  add_executable(rtk_velocity_guard_test tests/rtk_velocity_guard_test.cpp)
  target_include_directories(rtk_velocity_guard_test PRIVATE
    include ${EIGEN3_INCLUDE_DIRS})
  target_link_libraries(rtk_velocity_guard_test backend_core)
  add_test(NAME rtk_velocity_guard_test COMMAND rtk_velocity_guard_test)

  add_executable(rigid_relocalization_monitor_test
    tests/rigid_relocalization_monitor_test.cpp)
  target_include_directories(rigid_relocalization_monitor_test PRIVATE
    include ${EIGEN3_INCLUDE_DIRS})
  target_link_libraries(rigid_relocalization_monitor_test backend_core)
  add_test(NAME rigid_relocalization_monitor_test
    COMMAND rigid_relocalization_monitor_test)

  add_executable(rtk_fusion_diagnostics_test
    tests/rtk_fusion_diagnostics_test.cpp)
  target_include_directories(rtk_fusion_diagnostics_test PRIVATE
    include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
  target_link_libraries(rtk_fusion_diagnostics_test backend_core)
  add_test(NAME rtk_fusion_diagnostics_test COMMAND rtk_fusion_diagnostics_test)
endif()

install(TARGETS fastlivo_mapping DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz_cfg DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS scripts/play_all_bags.sh scripts/generate_mine_calibration.py
        scripts/prepare_wuhu_bag.py scripts/evaluate_wuhu_lio.py
        scripts/validate_backend_keyframes.py
        scripts/validate_odometry_pose_graph.py
        scripts/validate_loop_candidates.py
        scripts/validate_loop_registration.py
        scripts/validate_loop_verification.py
        scripts/validate_loop_pose_graph.py
        scripts/validate_optimized_global_map.py
        scripts/validate_rtk_fusion.py
        scripts/validate_rtk_diagnostics.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
