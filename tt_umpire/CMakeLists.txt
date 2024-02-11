cmake_minimum_required(VERSION 3.8)
project(tt_umpire)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

include(FetchContent)
FetchContent_Declare(
  rosbag
  URL https://github.com/gstavrinos/ai_msc_robotic_systems_assignments/releases/download/data-from-rosbag/ai_msc_robotic_systems_rosbag.tar.gz
)

FetchContent_MakeAvailable(rosbag)

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

add_executable(${PROJECT_NAME} src/tt_umpire.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp tf2_ros std_srvs nav_msgs geometry_msgs lifecycle_msgs)

install(DIRECTORY scripts/
      DESTINATION lib/${PROJECT_NAME}
      FILES_MATCHING
      PATTERN "*.py"
      PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME})

ament_package()
