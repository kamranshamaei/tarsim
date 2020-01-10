project (configParserLibProj)

include_directories(
    .
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/actorsRigidBody
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/node
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/object
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc/protobufs
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/fileSystem/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/timers/inc
    ${THIRDPARTY_SOURCE_DIRECTORY}/eigen/3.3.4/Eigen
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/collisionDetection
    ${CMAKE_PROTOBUF_OUTPUT_DIRECTORY}
    )

set(FILE_CONFIG_PARSER_HDRS 
    configParser.h
    )
    
set(FILE_CONFIG_PARSER_SRCS 
    configParser.cpp
    )
    

add_library(configParser ${FILE_CONFIG_PARSER_SRCS} ${FILE_CONFIG_PARSER_HDRS})
target_link_libraries(configParser fileSystem simProto node)