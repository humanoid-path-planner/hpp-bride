cmake_minimum_required(VERSION 2.8.3)
project(hpp_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
    dynamic_reconfigure 
	std_msgs
	std_srvs
	sensor_msgs
	geometry_msgs
	trajectory_msgs
)

# protected region additional user defined REQUIREMENTS on begin #
include (cmake/base.cmake)
set (CXX_DISABLE_WERROR True)
set (PROJECT_DESCRIPTION Implementation of a ROS node integrating hpp-core library)
set (PROJECT_URL https://github.com/humanoid-path-planner/hpp-bride)
setup_project ()
add_required_dependency ("hpp-core >= 3")
add_required_dependency ("hpp-model-urdf >= 3")
# protected region additional user defined REQUIREMENTS end #

generate_dynamic_reconfigure_options(cfg/hpp.cfg )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    common/src
  CATKIN_DEPENDS 
  	roscpp
    dynamic_reconfigure
	std_msgs
	std_srvs
	sensor_msgs
	geometry_msgs
	trajectory_msgs
   
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(common/include)
include_directories(common/src)
include_directories(${catkin_INCLUDE_DIRS})


## Declare a cpp executable
add_executable(hpp ros/src/hpp_ros.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(hpp hpp_gencfg)
add_dependencies(hpp ${hpp_node_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(hpp
  ${catkin_LIBRARIES}
)

# protected region additional user defined BUILD STATEMENTS on begin #
pkg_config_use_dependency (hpp hpp-core)
pkg_config_use_dependency (hpp hpp-model-urdf)
# protected region additional user defined BUILD STATEMENTS end #


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS hpp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# protected region user Cmake macros on begin #
setup_project_finalize ()
# protected region user Cmake macros end #
