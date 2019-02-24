project (JointSenderProj)


include_directories(
    ./    
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/msgQClient/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/logging/logClient/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/fileSystem/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/msgQClient/inc
    )

set(FILE_CLIENT_HDRS 
    ./eitOsMsgClientSender.h
    )
    
set(FILE_CLIENT_SRCS 
    ./eitOsMsgClientSender.cpp
    )
    
set(FILE_TEST_CLIENT_SRCS 
    unittests/main.cpp
    )

add_library(eitOsMsgClientSender ${FILE_CLIENT_SRCS} ${FILE_CLIENT_HDRS})
target_link_libraries(eitOsMsgClientSender)

if (EIT_UNIT_TEST_BUILD)
    #add_executable(eitOsMsgClientSenderUnitTest ${FILE_TEST_CLIENT_SRCS})
    #target_link_libraries(eitOsMsgClientSenderUnitTest threadUtils logClient eitOsMsgClientSender)
endif()

INSTALL(FILES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libeitOsMsgClientSender.so DESTINATION ./user/client/lib)

