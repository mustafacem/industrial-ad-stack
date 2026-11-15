cmake_minimum_required(VERSION 3.8)
project(iad_gz_sim)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(ZLIB REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# ---- site world model (seeded quarry / port), shared by the compiler, GNSS effects and the evaluators
add_library(iad_gz_world SHARED src/world/world.cpp src/world/scenarios.cpp src/world/maps.cpp src/world/gnss_model.cpp)
target_include_directories(iad_gz_world PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_options(iad_gz_world PRIVATE -O3)
target_link_libraries(iad_gz_world Eigen3::Eigen OpenMP::OpenMP_CXX)
ament_target_dependencies(iad_gz_world sensor_msgs)
install(TARGETS iad_gz_world EXPORT export_iad_gz_sim LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_iad_gz_sim HAS_LIBRARY_TARGET)
ament_export_dependencies(sensor_msgs Eigen3 OpenMP)

# ---- world compiler, run at build time for both sites
add_executable(world_compiler src/compiler/world_compiler.cpp)
target_link_libraries(world_compiler iad_gz_world ZLIB::ZLIB)
install(TARGETS world_compiler DESTINATION lib/${PROJECT_NAME})
set(WORLDS_OUT ${CMAKE_CURRENT_BINARY_DIR}/worlds)
foreach(site quarry port)
  add_custom_command(
    OUTPUT ${WORLDS_OUT}/${site}/world.sdf
    COMMAND world_compiler ${site} ${WORLDS_OUT}/${site} 7
    DEPENDS world_compiler iad_gz_world ${CMAKE_CURRENT_SOURCE_DIR}/src/world/scenarios.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/world/world.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/world/maps.cpp
    COMMENT "Compiling Gazebo world '${site}'")
  list(APPEND WORLD_FILES ${WORLDS_OUT}/${site}/world.sdf)
endforeach()
add_custom_target(compile_worlds ALL DEPENDS ${WORLD_FILES})
install(DIRECTORY ${WORLDS_OUT}/ DESTINATION share/${PROJECT_NAME}/worlds)

# ---- gz-sim system plugins
add_library(iad_gz_traffic SHARED src/plugins/traffic.cpp)
target_include_directories(iad_gz_traffic PRIVATE include)
target_link_libraries(iad_gz_traffic gz-sim8::gz-sim8 gz-plugin2::register gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)
add_library(iad_gz_vehicle SHARED src/plugins/vehicle_drive.cpp)
target_include_directories(iad_gz_vehicle PRIVATE include)
target_link_libraries(iad_gz_vehicle gz-sim8::gz-sim8 gz-plugin2::register gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)
ament_target_dependencies(iad_gz_vehicle rclcpp ackermann_msgs autoware_vehicle_msgs geometry_msgs nav_msgs std_msgs tf2_ros)
install(TARGETS iad_gz_traffic iad_gz_vehicle LIBRARY DESTINATION lib)

# ---- ROS nodes
add_executable(gnss_effects src/nodes/gnss_effects.cpp)
target_link_libraries(gnss_effects iad_gz_world)
ament_target_dependencies(gnss_effects rclcpp sensor_msgs std_msgs)
install(TARGETS gnss_effects DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/vehicle_description scripts/route_driver scripts/sim_supervisor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gz_sim test/test_gz_sim.cpp)
  target_link_libraries(test_gz_sim iad_gz_world)
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")
ament_package()
