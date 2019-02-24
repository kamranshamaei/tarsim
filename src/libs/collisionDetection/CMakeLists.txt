project (CollisionDetectionProj)


include_directories(
    .
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc/protobufs
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/logging/logClient/inc
    ${THIRDPARTY_SOURCE_DIRECTORY}/eigen/3.3.4/Eigen
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc/protobufs
    )

set(FILE_HDRS 
    boundingBoxBase.h
    boundingBoxCapsule.h
    boundingBoxSphere.h
    boundingBoxCuboid.h
    collisionDetection.h
    )
    
set(FILE_SRCS 
    boundingBoxBase.cpp
    boundingBoxCapsule.cpp
    boundingBoxSphere.cpp
    boundingBoxCuboid.cpp
    collisionDetection.cpp
    )

add_library(collisionDetection ${FILE_SRCS} ${FILE_HDRS})
target_link_libraries(collisionDetection logClient)
