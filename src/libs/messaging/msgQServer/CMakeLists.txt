project (MsgQReceiverLibProj)

include_directories(
    ./inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/logging/logClient/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/msgQClient/inc
    )


set(FILE_HDRS 
    inc/msgQServer.h
    )
    
set(FILE_SRCS 
    src/msgQServer.cpp
    )
    
add_library(msgQServer ${FILE_SRCS} ${FILE_HDRS})

target_link_libraries(msgQServer rt pthread)

