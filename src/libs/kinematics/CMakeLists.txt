project(PROJECT_KINEMATICS)

include_directories(
    .
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/configParser
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc/protobufs
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/actorsRigidBody
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/node
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/collisionDetection
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com/server
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/object
    )

set(FILE_HDRS 
    kinematics.h
    )
    
set(FILE_SRCS
    kinematics.cpp
    )

add_library(kinematics ${FILE_SRCS} ${FILE_HDRS})
target_link_libraries(kinematics node object eitServer configParser collisionDetection)
