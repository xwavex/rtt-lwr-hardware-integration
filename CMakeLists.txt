cmake_minimum_required(VERSION 2.8.3)
project(rtt-gazebo-lwr-integration)

execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=native --cflags OUTPUT_VARIABLE XENO_NATIVE_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=native --ldflags OUTPUT_VARIABLE XENO_NATIVE_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -O3 -fPIC -DHAVE_SOCKLEN_T")
#list(APPEND CMAKE_CXX_FLAGS "-std=c++11")
## Find catkin macros and libraries
#find_package(catkin REQUIRED COMPONENTS 
#    rtt_ros 
#    cmake_modules 
#    sensor_msgs 
#    message_generation 
#    std_msgs
#)
#
#catkin_package()

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba rtt-typekit)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos-RTT. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

#find_package(OROCOS-OCL REQUIRED COMPONENTS ocl-deployment ocl-taskbrowser ocl-logging)

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )


set ( CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/Modules/  )
find_package(Eigen REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

find_package(RCI REQUIRED)
message(STATUS "RCI version: ${RCI_VERSION}")	
include_directories(BEFORE SYSTEM ${RCI_INCLUDE_DIRS})	
add_definitions(${RCI_CFLAGS})
link_directories(${RCI_LIBRARY_DIR})

find_package(RSC REQUIRED)
message(STATUS "RSC version: ${RSC_VERSION}")	
include_directories(BEFORE SYSTEM ${RSC_INCLUDE_DIRS})	
add_definitions(${RSC_CFLAGS})
link_directories(${RSC_LIBRARY_DIR})

#find_package(orocos_kdl REQUIRED)

find_package(Boost COMPONENTS system filesystem thread REQUIRED)

find_package(RTnet REQUIRED)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${orocos_kdl_LIBRARY_DIRS}
  ${PROJECT_NAME_BIN}/lib 
  ${Boost_LIBRARY_DIR}
  )
  
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIR}
  ${RTnet_INCLUDE_DIR}
  )
 
 ADD_LIBRARY(FRI_RT
  src/friremote_rt.cpp  
  src/friudp_rt.cpp  
)

orocos_use_package( ocl-taskbrowser )
orocos_use_package( ocl-deployment )
orocos_use_package( ocl-logging )
orocos_use_package( orocos-rtt-corba )
#orocos_use_package( rtt-typekit )

#add_definitions(${OROCOS-RTT_DEFINITIONS})

if(${OROCOS_TARGET} STREQUAL "xenomai" )
  find_package(Xenomai REQUIRED)
  set_property(DIRECTORY ${PROJECT_SOURCE_DIR} APPEND PROPERTY COMPILE_DEFINITIONS HAVE_RTNET XENOMAI)
endif()
 message(STATUS "HELP ${RTNET_LIBRARIES}\n---------------------\n")
## Default component
  
orocos_component(lwr_fri src/lwr_fri_component.cpp)
set_target_properties(lwr_fri PROPERTIES COMPILE_DEFINITIONS RTT_COMPONENT)
target_link_libraries(lwr_fri 
  ${orocos_kdl_LIBRARIES} 
  ${GAZEBO_LIBRARIES} 
  ${OROCOS-RTT_LIBRARIES}
  ${OROCOS-RTT_LIBRARIES} 
  ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} 
  ${OROCOS-RTT_RTT-TRANSPORT-CORBA_LIBRARY}
  ${OROCOS-RTT_TYPEKIT_LIBRARIES}
  ${USE_OROCOS_LIBRARIES}
  ${Boost_LIBRARIES}
  ${RSC_LIBRARIES}
  ${RCI_LIBRARIES}
   FRI_RT
   ${XENO_NATIVE_LDFLAGS}
   -lrtdm)

orocos_generate_package()
