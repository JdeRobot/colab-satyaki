cmake_minimum_required(VERSION 2.8.3)
project(ros_compat)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Num.msg
   Pose3d.msg
   Motors.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_compat
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#added 
SET(INTERFACES_CPP_DIR ~/Documents/JdeRobot/src/stable/interfaces/cpp/)
SET(LIBS_DIR ~/Documents/JdeRobot/src/stable/interfaces)
SET(LIBS_DIR2 ~/Documents/JdeRobot/src/stable/libs)
SET(easyiceconfig_INCLUDE_DIRS ${LIBS_DIR2}/easyiceconfig_cpp/include)

FIND_PATH( Ice_INCLUDE_DIR NAMES Ice/Ice.h  PATHS ENV C++LIB ENV)
IF( Ice_INCLUDE_DIR )
    FIND_LIBRARY( Ice_LIBRARY1 NAMES Ice PATHS ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64 )
    FIND_LIBRARY( Ice_LIBRARY2 NAMES IceUtil PATHS ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64)
    SET (Ice_LIBRARIES ${Ice_LIBRARY1} ${Ice_LIBRARY2})
    IF( Ice_LIBRARIES )
                MESSAGE ("-- Ice found at ${Ice_LIBRARIES}")
		#TARGET_LINK_LIBRARIES(JdeRobotCppInterfaces ${Ice_LIBRARY2})
    ENDIF( Ice_LIBRARIES )
ENDIF(Ice_INCLUDE_DIR)
IF(NOT Ice_LIBRARIES)
                MESSAGE ("*** Ice not found")
ENDIF()

#include_directories(
#    ${INTERFACES_CPP_DIR}
#    ${LIBS_DIR}
#    ${LIBS_DIR2}
#    ${CMAKE_CURRENT_SOURCE_DIR}
#    ${gtkmm_INCLUDE_DIRS}
#   ${libglademm_INCLUDE_DIRS}
#    ${gtkglextmm_INCLUDE_DIRS}
#    ${ZLIB_INCLUDE_DIRS}
#    ${resourcelocator_INCLUDE_DIRS}
#    
#)

add_executable(camdriver src/CameraDriver.cpp)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(camdriver ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES})
add_dependencies(camdriver ros_compat_generate_messages_cpp)

add_executable(camclient src/CameraClient.cpp)
include_directories(${OpenCV_INCLUDE_DIRS}
		${INTERFACES_CPP_DIR}
		${LIBS_DIR}
		${LIBS_DIR2}
		${LIBS_DIR2}/parallelIce
		${Ice_INCLUDE_DIR}
		${easyiceconfig_INCLUDE_DIRS})
target_link_libraries(camclient 
		${catkin_LIBRARIES}
		${Ice_LIBRARIES}
		${ZeroCIce_LIBRARIES} 
		${OpenCV_LIBS} 
		${OpenCV_LIBRARIES} 
		${LIBS_DIR2}/visionlib/colorspaces/libcolorspacesmm.so
		${LIBS_DIR2}/parallelIce/libparallelIce.so
		${LIBS_DIR}/libJderobotInterfaces.so
		${LIBS_DIR2}/easyiceconfig_cpp/libeasyiceconfig.so
		)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(laserTurtlebotJde SHARED src/LaserDriver.cc)
target_link_libraries(laserTurtlebotJde 
		RayPlugin
		${catkin_LIBRARIES} 
		${GAZEBO_LIBRARIES} 
		${Ice_LIBRARIES}
		${LIBS_DIR}/libJderobotInterfaces.so
		${LIBS_DIR2}/easyiceconfig_cpp/libeasyiceconfig.so
		${LIBS_DIR2}/visionlib/colorspaces/libcolorspacesmm.so)

add_library(pose3dTurtlebotJde SHARED src/Pose3dDriver.cc)
target_link_libraries(pose3dTurtlebotJde 
		#RayPlugin
		${catkin_LIBRARIES} 
		${GAZEBO_LIBRARIES} 
		${Ice_LIBRARIES}
		${LIBS_DIR}/libJderobotInterfaces.so
		${LIBS_DIR2}/easyiceconfig_cpp/libeasyiceconfig.so)
		#${LIBS_DIR2}/visionlib/colorspaces/libcolorspacesmm.so)

add_library(camera_dumpTurtlebotJde SHARED src/CameraDump.cc)
target_link_libraries(camera_dumpTurtlebotJde
		CameraPlugin
		${catkin_LIBRARIES} 
		${GAZEBO_LIBRARIES} 
		${Ice_LIBRARIES}
		${OpenCV_LIBS} 
		${OpenCV_LIBRARIES} 
		${LIBS_DIR2}/visionlib/colorspaces/libcolorspacesmm.so
		${LIBS_DIR2}/parallelIce/libparallelIce.so
		${LIBS_DIR}/libJderobotInterfaces.so
		${LIBS_DIR2}/easyiceconfig_cpp/libeasyiceconfig.so)

add_library(motorsTurtlebotJde SHARED src/MotorDriver.cc)
target_link_libraries(motorsTurtlebotJde
		RayPlugin
		${catkin_LIBRARIES} 
		${GAZEBO_LIBRARIES} 
		${Ice_LIBRARIES}
		#${OpenCV_LIBS} 
		#${OpenCV_LIBRARIES} 
		${LIBS_DIR2}/visionlib/colorspaces/libcolorspacesmm.so
		${LIBS_DIR2}/parallelIce/libparallelIce.so
		${LIBS_DIR}/libJderobotInterfaces.so
		${LIBS_DIR2}/easyiceconfig_cpp/libeasyiceconfig.so)
