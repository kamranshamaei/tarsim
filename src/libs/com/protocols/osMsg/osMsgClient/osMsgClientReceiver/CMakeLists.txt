project (ReceiveSimReplyProj)
message("--------------------------------------------------------------------")
message("                  Configuring ReceiveSimReply                       ")
message("--------------------------------------------------------------------")

message ("Source   Directory: ${CMAKE_SOURCE_DIR}")
message ("External Directory: ${EXTERNAL_DIRECTORY}")


include_directories(
    .
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/msgQClient/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/logging/logClient/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/msgQServer/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/timers/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    )



set(FILE_SRV_HDRS 
    eitOsMsgClientReceiver.h
    )
    
set(FILE_SRV_SRCS 
    eitOsMsgClientReceiver.cpp
    )
       
add_library(eitOsMsgClientReceiver ${FILE_SRV_SRCS} ${FILE_SRV_HDRS})
target_link_libraries(eitOsMsgClientReceiver msgQServer timer)

INSTALL(FILES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libeitOsMsgClientReceiver.so DESTINATION ./user/client/lib)
INSTALL(FILES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libmsgQServer.so DESTINATION ./user/client/lib)
INSTALL(FILES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libtimer.so DESTINATION ./user/client/lib)
INSTALL(FILES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/liblogClient.so DESTINATION ./user/client/lib)
INSTALL(FILES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libmsgQClient.so DESTINATION ./user/client/lib)

