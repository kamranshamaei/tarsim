project (TimerUtilProj)

message (${CMAKE_SOURCE_DIR})
include_directories(
    ./inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/msgQClient/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/logging/logClient/inc
    )

set(FILE_SRV_HDRS 
    inc/timerUtils.h
    )
    
set(FILE_SRV_SRCS 
    src/timerUtils.cpp
    )
    
add_library(timer ${FILE_SRV_SRCS} ${FILE_SRV_HDRS})
target_link_libraries(timer rt logClient)
