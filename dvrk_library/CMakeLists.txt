cmake_minimum_required(VERSION 2.8.3)
project(dvrk_library)

find_package(catkin REQUIRED COMPONENTS
cisst_ros_bridge
roscpp
std_msgs
roslib
)


#add_definitions(-std=c++11)
find_package(cmake_modules REQUIRED)

find_package( PythonLibs 2.7 REQUIRED )
include_directories( ${PYTHON_INCLUDE_DIRS} )

find_package(Eigen REQUIRED)

catkin_python_setup()
catkin_package(
  INCLUDE_DIRS include
 # LIBRARIES laprotek_comm
 #CATKIN_DEPENDS cisst_ros_bridge roscpp roslib
 # CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
 # DEPENDS system_lib
)

set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstVectorQt
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  cisstRobot
  cisstNumerical
  cisstQt
  )

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})
include (${CISST_USE_FILE})
cisst_find_saw_component (sawRobotIO1394 REQUIRED)
include_directories (${sawRobotIO1394_INCLUDE_DIR})
link_directories (${sawRobotIO1394_LIBRARY_DIR})

find_package( Boost COMPONENTS python REQUIRED )
include_directories( ${Boost_INCLUDE_DIR} )

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
include_directories(include)

add_executable(ecm src/ecm.cpp)
add_executable(psm src/psm.cpp)

add_library( dvrk_library SHARED src/dvrk_library.cpp )

target_link_libraries(dvrk_library
   ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${sawRobotIO1394_LIBRARIES}
 )

target_link_libraries(psm
   ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${sawRobotIO1394_LIBRARIES}
 )
cisst_target_link_libraries (psm ${REQUIRED_CISST_LIBRARIES})

set_target_properties(dvrk_library PROPERTIES PREFIX ""
 # LIBRARY_OUTPUT_DIRECTORY /usr/lib/python2.7/dist-packages)
LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
