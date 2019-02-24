project (LoggerClientProj)

include_directories(
    ./inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc 
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/msgQClient/inc    
    )

set(FILE_CLIENT_HDRS 
    inc/logClient.h
    )
    
set(FILE_CLIENT_SRCS 
    src/logClient.cpp
    )
    

add_library(logClient ${FILE_CLIENT_SRCS} ${FILE_CLIENT_HDRS})

target_link_libraries(logClient msgQClient)
