project (ObjectProj)


find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${CMAKE_PROTOBUF_OUTPUT_DIRECTORY}
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/node
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/actorsRigidBody
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/logging/logClient/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/msgQClient/inc 
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/collisionDetection
    ${CMAKE_PROTOBUF_OUTPUT_DIRECTORY} 
    
    ${THIRDPARTY_SOURCE_DIRECTORY}/eigen/3.3.4/Eigen
    ${VTK_INCLUDE_DIRS}
    )

set(FILE_NODE_HDRS 
    object.h
    )
    
set(FILE_NODE_SRCS 
    object.cpp
    )
    
add_library(object ${FILE_NODE_SRCS} ${FILE_NODE_HDRS})
target_link_libraries(object node logClient ${VTK_LIBRARIES} simProto actorsRigidBody)
