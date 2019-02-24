project (TarsimClientExposedProj)

include_directories(
    .
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com/client
    )

set(FILE_HDRS 
    tarsimClientExposed.h
    )
    
set(FILE_SRCS 
    tarsimClientExposed.cpp
    )
    
add_library(tarsimClientExposed ${FILE_SRCS} ${FILE_HDRS})
target_link_libraries(tarsimClientExposed tarsimClient)