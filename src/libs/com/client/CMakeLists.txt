project (TarsimClientProj)

include_directories(
    .
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com/protocols/osMsg/osMsgClient/osMsgClientReceiver
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com/protocols/osMsg/osMsgClient/osMsgClientSender
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/msgQClient/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/msgQServer/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/timers/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/fileSystem/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/msgQClient/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/msgQServer/inc
    )

set(FILE_HDRS 
    tarsimClient.h
    )
    
set(FILE_SRCS 
    tarsimClient.cpp
    )
    
add_library(tarsimClient ${FILE_SRCS} ${FILE_HDRS})
target_link_libraries(tarsimClient eitOsMsgClientReceiver eitOsMsgClientSender)
target_include_directories(tarsimClient PUBLIC 
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com/protocols/osMsg/osMsgClient/osMsgClientReceiver
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com/protocols/osMsg/osMsgClient/osMsgClientSender)


if(GENERATE_WRAPPER)
    add_subdirectory(wrappers)    
endif(GENERATE_WRAPPER) 



INSTALL(FILES ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com/client/tarsimClient.h DESTINATION ./user/client/inc)
INSTALL(FILES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libtarsimClient.so DESTINATION ./user/client/lib)
INSTALL(FILES ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc/ipcMessages.h DESTINATION ./user/client/inc)

    
    
    



