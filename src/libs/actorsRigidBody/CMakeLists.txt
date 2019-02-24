project (ActorsRigidBodyProj)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/node
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc/protobufs
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/logging/logClient/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/fileSystem/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/timers/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/msgQClient/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/collisionDetection
    ${CMAKE_PROTOBUF_OUTPUT_DIRECTORY}
    
    ${THIRDPARTY_SOURCE_DIRECTORY}/eigen/3.3.4/Eigen
    ${VTK_INCLUDE_DIRS}
    )

set(FILE_ACTORS_RIGID_BODY_HDRS 
    actorsRigidBody.h
    )
    
set(FILE_RIGID_BODY_SRCS 
    actorsRigidBody.cpp
    )
    
add_library(actorsRigidBody ${FILE_RIGID_BODY_SRCS} ${FILE_RIGID_BODY_HDRS})
target_link_libraries(actorsRigidBody fileSystem logClient ${VTK_LIBRARIES})
