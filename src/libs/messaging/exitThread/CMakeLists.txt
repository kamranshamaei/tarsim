project (SendUserReplyProj)


include_directories(
    ./    
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/msgQClient/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/logging/logClient/inc
    )

set(FILE_CLIENT_HDRS 
    ./exitThread.h
    )
    
set(FILE_CLIENT_SRCS 
    ./exitThread.cpp
    )
    

add_library(exitThread ${FILE_CLIENT_SRCS} ${FILE_CLIENT_HDRS})
target_link_libraries(exitThread msgQClient logClient)


