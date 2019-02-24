project (TarsimProj)

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
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/object
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/kinematics
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/configParser
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/gui
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/scenes
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com 
    ${THIRDPARTY_SOURCE_DIRECTORY}/eigen/3.3.4/Eigen
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/collisionDetection
    ${CMAKE_PROTOBUF_OUTPUT_DIRECTORY}
    ${VTK_DIR}
    ${VTK_INCLUDE_DIRS}
    ${PROTOBUF_DIR}/include
    )

set(FILE_HDRS 
    tarsim.h
    )
    
set(FILE_SRCS 
    tarsim.cpp
    )
    
add_library(tarsimLib ${FILE_SRCS} ${FILE_HDRS})

target_link_libraries(tarsimLib 
    ${VTK_LIBRARIES} 
    configParser 
    kinematics 
    eitOsMsgServerReceiver 
    threadUtils 
    gui
    exitThread
    logServer)

INSTALL(FILES ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/tarsim/tarsim.h DESTINATION ./user/server/inc)
INSTALL(DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} DESTINATION ./user/server)
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/example DESTINATION ./user/server)