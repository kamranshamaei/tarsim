project (MsgQClientProj)

include_directories(
    ./inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/logging/logClient/inc
    )


set(FILE_HDRS 
    inc/msgQClient.h
    )
    
set(FILE_SRCS 
    src/msgQClient.cpp
    )
    
add_library(msgQClient ${FILE_SRCS} ${FILE_HDRS})

target_link_libraries(msgQClient rt pthread)

