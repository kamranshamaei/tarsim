project (NodeProj)

find_package(Eigen3 REQUIRED)
find_package(VTK REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${CMAKE_PROTOBUF_OUTPUT_DIRECTORY}
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/actorsRigidBody
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/logging/logClient/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/msgQClient/inc 
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/collisionDetection
    ${CMAKE_PROTOBUF_OUTPUT_DIRECTORY}     
    ${VTK_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    )

set(FILE_NODE_HDRS 
    node.h
    )
    
set(FILE_NODE_SRCS 
    node.cpp
    )
    
add_library(node ${FILE_NODE_SRCS} ${FILE_NODE_HDRS})
target_link_libraries(node logClient ${VTK_LIBRARIES} simProto actorsRigidBody Eigen3::Eigen)
# vtk_module_autoinit is needed
vtk_module_autoinit(
    TARGETS node
    MODULES ${VTK_LIBRARIES}
    )