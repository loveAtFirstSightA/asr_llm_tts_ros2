cmake_minimum_required(VERSION 3.8)
project(interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(spdlog REQUIRED)

include_directories(include)

set(executable_name interface)

set(library_name ${executable_names}_core)

set(dependencies
  rclcpp
  std_msgs
  spdlog
)

add_executable(${executable_name} src/main.cpp)
target_include_directories(${executable_name} PRIVATE include)
ament_target_dependencies(${executable_name} ${dependencies})

add_library(${library_name} src/interface.cpp)
target_include_directories(${library_name} PRIVATE include)
ament_target_dependencies(${library_name} ${dependencies})

target_link_libraries(${executable_name} ${library_name} fmt::fmt)

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

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_package()
