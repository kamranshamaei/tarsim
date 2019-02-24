project (LogServerProj)

message (${CMAKE_SOURCE_DIR})
include_directories(
    ./inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/logging/logClient/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/msgQServer/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/utilities/threadUtils/inc
    
    )

set(FILE_SRV_HDRS 
    inc/logServer.h
    )
    
set(FILE_SRV_SRCS 
    src/logServer.cpp
    )
    

add_library(logServer  ${FILE_SRV_SRCS} ${FILE_SRV_HDRS})
target_link_libraries(logServer msgQServer)
