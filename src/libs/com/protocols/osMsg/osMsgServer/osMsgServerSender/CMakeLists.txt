project (EitOsMsgServerReceiverProj)


include_directories(
    .    
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/msgQClient/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/logging/logClient/inc
    )

set(FILE_CLIENT_HDRS 
    eitOsMsgServerSender.h
    )
    
set(FILE_CLIENT_SRCS 
    eitOsMsgServerSender.cpp
    )
    

add_library(eitOsMsgServerSender  ${FILE_CLIENT_SRCS} ${FILE_CLIENT_HDRS})
target_link_libraries(eitOsMsgServerSender)


