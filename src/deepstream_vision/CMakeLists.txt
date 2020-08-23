cmake_minimum_required(VERSION 3.0.2)
project(deepstream_vision)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fmax-errors=1")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package( OpenCV REQUIRED )

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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES deepstream_vision
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/deepstream_vision/
  /opt/nvidia/deepstream/deepstream/sources/includes/
  /usr/include/opencv4/opencv
  /usr/include/opencv4
  /usr/include/gstreamer-1.0
  /usr/include/glib-2.0
  /usr/lib/aarch64-linux-gnu/glib-2.0/include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  /opt/nvidia/deepstream/deepstream/lib
)

set(GST_LIBRARIES gstreamer-1.0 gobject-2.0 glib-2.0)
set(DS_LIBRARIES nvdsgst_meta nvds_meta)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/deepstream_vision.cpp
# )

add_library(common src/common.cpp)
target_link_libraries(common ${GST_LIBRARIES} )

add_library(simple_pipeline src/simple_pipeline.cpp)
target_link_libraries(simple_pipeline ${GST_LIBRARIES} common)

add_library(simple_visualize_sink src/simple_visualize_sink.cpp)
target_link_libraries(simple_visualize_sink ${GST_LIBRARIES} common)

add_library(opencv_dewarp_source src/opencv_dewarp_source.cpp)
target_link_libraries(opencv_dewarp_source ${GST_LIBRARIES} ${OpenCV_LIBS} common)

add_library(ros_image_source src/ros_image_source.cpp)
target_link_libraries(ros_image_source ${GST_LIBRARIES} ${OpenCV_LIBS} common ${catkin_LIBRARIES})

add_library(memory_mover src/memory_mover.cpp)
target_link_libraries(simple_visualize_sink ${GST_LIBRARIES} ${DS_LIBRARIES} common)

add_library(nvinfer_node src/nvinfer_node.cpp)
target_link_libraries(nvinfer_node ${GST_LIBRARIES} ${DS_LIBRARIES} common)

# CFLAGS+= -I$(DS_SRC_PATH)/sources/includes \
# 	  -I/usr/local/cuda-$(CUDA_VER)/include

# CFLAGS+= -Wall -std=c++11 -shared -fPIC

# LIBS+= -lnvinfer -lnvparsers -L/usr/local/cuda-$(CUDA_VER)/lib64 -lcudart -lcublas

# LFLAGS:= -Wl,--start-group $(LIBS) -Wl,--end-group
add_library(nvdsinfer_custombboxparser_yolov3_tlt src/nvdsinfer_custombboxparser_yolov3_tlt.cpp)
target_include_directories(nvdsinfer_custombboxparser_yolov3_tlt PUBLIC /usr/local/cuda-10.2/include)
set_target_properties(${TARGET} PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -std=c++11 -shared -fPIC")
target_link_directories(nvdsinfer_custombboxparser_yolov3_tlt PUBLIC /usr/local/cuda-10.2/lib64)
target_link_libraries(nvdsinfer_custombboxparser_yolov3_tlt nvinfer nvparsers cudart cublas)


option(LOWERCAM "LOWER CAMERA OR UPPER CAMERA" ON)

if(LOWERCAM)
  message("-- Using lower camera parameters")
  add_compile_definitions(LOWER)
else()
  message("-- Using upper camera parameters")
  add_compile_definitions(UPPER)
endif()

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/deepstream_vision_node.cpp)
add_executable(test_basic_pipeline src/test_basic_pipeline.cpp)
add_executable(test_nvinfer_pipeline src/test_nvinfer_pipeline.cpp)
add_executable(test_ros_nvinfer_pipeline src/test_ros_nvinfer_pipeline.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(test_basic_pipeline
  simple_pipeline simple_visualize_sink opencv_dewarp_source memory_mover common
)

target_link_libraries(test_nvinfer_pipeline
  simple_pipeline simple_visualize_sink opencv_dewarp_source memory_mover common nvinfer_node
)

target_link_libraries(test_ros_nvinfer_pipeline
  simple_pipeline simple_visualize_sink ros_image_source memory_mover common nvinfer_node ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_deepstream_vision.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
