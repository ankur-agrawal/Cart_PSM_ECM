cmake_minimum_required(VERSION 2.8.3)
project(dvrk_plugins)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_manager
  control_toolbox
  pluginlib
  hardware_interface
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
  DEPENDS roscpp
  controller_manager
  control_toolbox
  pluginlib
  hardware_interface
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} simulation_interface
)
link_directories(${GAZEBO_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})

include_directories(include)
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/dvrk_gazebo_control_plugin_new.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(simulation_interface src/simulation_interface.cpp)
target_link_libraries(simulation_interface ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS ${PROJECT_NAME} simulation_interface
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES simulation_interface.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
