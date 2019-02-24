project(PROJECT_FILESYSTEM)

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

include_directories(
    ./inc
    ${PROTOBUF_INCLUDE_DIRS}
    )

set(FILE_HDRS 
    ./inc/fileSystem.h
    )
    
set(FILE_SRCS 
    ./src/fileSystem.cpp
    )

add_library(fileSystem  ${FILE_SRCS} ${FILE_HDRS})
target_link_libraries(fileSystem ${PROTOBUF_LIBRARIES})


