include_directories(
    ${path_to_simulatorMessages_h}
    ${path_to_tarsimClient_h}
    )

set(FILE_HDRS 
    )
    
set(FILE_SRCS 
    ./clientSample.cpp
    )
    
    
add_executable(clientSample ${FILE_SRCS}  ${FILE_HDRS})
target_link_libraries(clientSample tarsimClient)

