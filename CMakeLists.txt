cmake_minimum_required(VERSION 2.8.3)
project(rtt-lwr-hardware-integration)

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

if(${OROCOS_TARGET} STREQUAL "xenomai" )
  find_package(Xenomai REQUIRED)
  set_property(DIRECTORY ${PROJECT_SOURCE_DIR} APPEND PROPERTY COMPILE_DEFINITIONS XENOMAI)
endif()

find_package(RTnet)
if(${RTNET_FOUND})
  set_property(DIRECTORY ${PROJECT_SOURCE_DIR} APPEND PROPERTY COMPILE_DEFINITIONS HAVE_RTNET)
  set(RTLIB_LIBRARIES -lrtdm)
endif()

find_package(RST-RT REQUIRED)
message(STATUS "RST-RT version: ${RST-RT_VERSION}")
include_directories(BEFORE SYSTEM ${RST-RT_INCLUDE_DIRS})
add_definitions(${RST-RT_CFLAGS})
link_directories(${RST-RT_LIBRARY_DIR} ${RST-RT_LIBRARY_DIRS})

find_package(PkgConfig REQUIRED)
pkg_search_module(TINYXML REQUIRED tinyxml)
if(NOT TINYXML_INCLUDE_DIR)
        find_path("/usr/include" tinyxml.h)
        find_library(TINYXML_LIBRARY NAMES tinyxml)
        set(TINYXML_LIBRARIES ${TINYXML_LIBRARY})
    set(TINYXML_INCLUDE_DIRS ${TINYXML_INCLUDE_DIR})
    set(TINYXML_LIBRARY_DIR "/usr/lib/x86_64-linux-gnu")
endif()
message(STATUS "TINYXML_VERSION: ${TINYXML_VERSION}")
include_directories(BEFORE SYSTEM ${TINYXML_INCLUDE_DIRS})
link_directories(${TINYXML_LIBRARY_DIR})
message(STATUS "TINYXML_LIBRARIES: ${TINYXML_LIBRARIES}")
message(STATUS "TINYXML_LIBRARY_DIR: ${TINYXML_LIBRARY_DIR}")
message(STATUS "TINYXML_INCLUDE_DIR: ${TINYXML_INCLUDE_DIRS}")
message(STATUS "TINYXML_LIBRARY: ${TINYXML_LIBRARY}")

#find_package(orocos_kdl REQUIRED)

find_package(Boost COMPONENTS system filesystem thread REQUIRED)

link_directories(
  ${orocos_kdl_LIBRARY_DIRS}
  ${PROJECT_NAME_BIN}/lib 
  ${Boost_LIBRARY_DIR}
  )
  
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${orocos_kdl_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIR}
  ${RTNET_INCLUDE_DIR}
  ${RST-RT_INCLUDE_DIRS}
  ${TINYXML_INCLUDE_DIRS}
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

## Default component
  
orocos_component(lwr_fri src/lwr_fri_component.cpp)
set_target_properties(lwr_fri PROPERTIES COMPILE_DEFINITIONS RTT_COMPONENT)
target_link_libraries(lwr_fri 
  ${orocos_kdl_LIBRARIES} 
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
   ${RTLIB_LIBRARIES}
   ${RST-RT_LIBRARIES}
    ${TINYXML_LIBRARIES}
   )

orocos_generate_package(INCLUDE_DIRS include)
