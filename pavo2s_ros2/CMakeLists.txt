cmake_minimum_required(VERSION 3.5)
project(pavo2s_ros)

set(CMAKE_BUILD_TYPE Release)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED system thread) 
include_directories(${Boost_INCLUDE_DIRS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(PAVO2S_SDK_PATH "./sdk")

FILE(GLOB PAVO2S_SDK_SRC 
 
  "${PAVO2S_SDK_PATH}/src/*.cpp"
)
include_directories(
  ${PAVO2S_SDK_PATH}/include
  ${PAVO2S_SDK_PATH}/src
  ${catkin_INCLUDE_DIRS}
)


add_executable(pavo2s_scan_node src/pavo2s_scan_node.cpp src/pavo2s_utils.cpp ${PAVO2S_SDK_SRC})
target_link_libraries(pavo2s_scan_node ${Boost_LIBRARIES} )
ament_target_dependencies(pavo2s_scan_node rclcpp std_msgs std_srvs sensor_msgs Boost)


add_executable(pavo2s_pcd_node src/pavo2s_pcd_node.cpp src/pavo2s_utils.cpp ${PAVO2S_SDK_SRC})
target_link_libraries(pavo2s_pcd_node ${Boost_LIBRARIES} )
ament_target_dependencies(pavo2s_pcd_node rclcpp std_msgs std_srvs sensor_msgs Boost)


install(TARGETS
pavo2s_pcd_node
pavo2s_scan_node
DESTINATION lib/${PROJECT_NAME}
  # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  # LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  # RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
