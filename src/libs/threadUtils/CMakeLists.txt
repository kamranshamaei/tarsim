project (ThreadutilsProj)


include_directories(
    ./inc
    ${CMAKE_SOURCE_DIR}/src/libs/logClient/inc
    )


set(FILE_HDRS 
    inc/threadUtilities.h
    )
    
set(FILE_SRCS 
    src/threadUtilities.cpp
    )
    
add_library(threadUtils ${FILE_SRCS} ${FILE_HDRS})

target_link_libraries(threadUtils)



