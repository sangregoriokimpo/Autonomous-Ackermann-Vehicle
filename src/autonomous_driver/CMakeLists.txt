cmake_minimum_required(VERSION 3.8)
project(autonomous_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(visualization_msgs)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(autonomous_driver src/autonomous_driver.cpp)

ament_target_dependencies(autonomous_driver
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
)

target_link_libraries(autonomous_driver ${PCL_LIBRARIES})

install(TARGETS autonomous_driver
  DESTINATION lib/${PROJECT_NAME})

add_executable(rgbd_driver src/rgbd_driver.cpp)
ament_target_dependencies(rgbd_driver
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  rgbd_driver
  DESTINATION lib/${PROJECT_NAME})


  add_executable(fusion_driver src/fusion_driver.cpp)
ament_target_dependencies(fusion_driver
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  visualization_msgs
  PCL  # if you link it explicitly elsewhere
)

install(TARGETS
  fusion_driver
  DESTINATION lib/${PROJECT_NAME})


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
  

ament_package()
