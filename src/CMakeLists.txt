# GENERAL INFO------------------------------------------------------------------
cmake_minimum_required(VERSION 3.0)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")

SET(PRODUCT_NAME "tarsim")
project(${PRODUCT_NAME})

SET(BUILD_SHARED_LIBS ON)

# OPTIONS ----------------------------------------------------------------------
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(TARSIM_LIBRARIES_SOURCE_DIRECTORY ${CMAKE_SOURCE_DIR}/libs)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/archive)
SET(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/${PRODUCT_NAME})
SET(path_to_tarsimClient_h ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com/client)
SET(path_to_simulatorMessages_h ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc)
SET(CMAKE_PROTOBUF_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/proto/")
SET(EIT_UNIT_TEST_BUILD false CACHE BOOL "Create Unit Tests")
SET(GENERATE_WRAPPER false CACHE BOOL "Generate wrapper")
SET(PYTHON_VERSION "2.7" CACHE STRING "Python version")
SET(BUILD_INCLUDE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/inc)

if( NOT CMAKE_BUILD_TYPE )
  SET( CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, 
options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE )
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
endif()

if( NOT RELATIVE_THIRDPARTY_SOURCE_DIRECTORY)
    SET(RELATIVE_THIRDPARTY_SOURCE_DIRECTORY 
    "${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty" 
    CACHE PATH "Relative Thirdparty Source Directory")
endif()
get_filename_component(THIRDPARTY_SOURCE_DIRECTORY 
    "${RELATIVE_THIRDPARTY_SOURCE_DIRECTORY}" ABSOLUTE)
unset(RELATIVE_THIRDPARTY_SOURCE_DIRECTORY CACHE)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(Protobuf REQUIRED)

include_directories (
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/fileSystem/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/logging/logClient/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/logging/logServer/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/msgQClient/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/msgQServer/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/exitThread/
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/threadUtils/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/timers/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc/protobufs
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/configParser
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/actorsRigidBody
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/node
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/kinematics
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/configParser
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/gui
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/scenes
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/tarsim 
    ${THIRDPARTY_SOURCE_DIRECTORY}/eigen/3.3.4/Eigen
    ${CMAKE_PROTOBUF_OUTPUT_DIRECTORY}
    ${VTK_DIR}
    ${VTK_INCLUDE_DIRS}
    ${PROTOBUF_DIR}/include
    )

add_subdirectory(config)
add_subdirectory(libs)
add_subdirectory(samples)

add_executable(${PRODUCT_NAME} ./simApp.cpp)
target_link_libraries(${PRODUCT_NAME} tarsimLib)

# INSTALL ----------------------------------------------------------------------
INSTALL(DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} DESTINATION .)
INSTALL(PROGRAMS ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PRODUCT_NAME} DESTINATION .)
INSTALL(PROGRAMS ${CMAKE_SOURCE_DIR}/setup.sh DESTINATION .)

# UNINSTALL --------------------------------------------------------------------



