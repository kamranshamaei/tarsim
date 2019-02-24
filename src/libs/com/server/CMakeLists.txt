project (EitServerProj)

include_directories(
    .
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com/protocols/osMsg/osMsgServer/osMsgServerReceiver
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com/protocols/osMsg/osMsgServer/osMsgServerSender
    )

set(FILE_SERVER_HDRS 
    eitServer.h
    )
    
set(FILE_SERVER_SRCS 
    eitServer.cpp
    )
    
add_library(eitServer ${FILE_SERVER_SRCS} ${FILE_SERVER_HDRS})
target_link_libraries(eitServer eitOsMsgServerReceiver)
target_include_directories(eitServer PUBLIC 
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com/protocols/osMsg/osMsgServer/osMsgServerReceiver
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com/protocols/osMsg/osMsgServer/osMsgServerSender)