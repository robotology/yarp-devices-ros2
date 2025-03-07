cmake_minimum_required(VERSION 3.17)
project(joint_states_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)

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

# let the compiler search for headers in the include folder
include_directories(include)

# define a library target called joint_states_client
add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)


ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  rcpputils
  rcutils
  sensor_msgs
)

# this line to exports the library
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

# install the include/joint_states_client directory to the install/include/joint_states_client
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# export information to downstream packages
ament_export_dependencies(
  rclcpp
  rcpputils
  rcutils
  sensor_msgs
)

ament_package()
