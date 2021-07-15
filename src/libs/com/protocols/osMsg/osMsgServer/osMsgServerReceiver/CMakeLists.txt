project (RobotJointsReceiverProj)
message("--------------------------------------------------------------------")
message("                  Configuring RobotJointsReceiver                   ")
message("--------------------------------------------------------------------")

message ("Source   Directory: ${CMAKE_SOURCE_DIR}")
message ("External Directory: ${EXTERNAL_DIRECTORY}")


include_directories(
    .
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/msgQClient/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/logging/logClient/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/messaging/msgQServer/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/utilities/threadUtils/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/utilities/timers/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com/protocols/osMsg/osMsgServer/osMsgServerReceiver
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com/protocols/osMsg/osMsgServer/osMsgServerSender
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/actorsRigidBody
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/configParser
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/node
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/object
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc/protobufs
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com/server
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/kinematics
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/scenes
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/gui
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/timers/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/collisionDetection
    )



set(FILE_SRV_HDRS 
    eitOsMsgServerReceiver.h
    )
    
set(FILE_SRV_SRCS 
    eitOsMsgServerReceiver.cpp
    )
       
add_library(eitOsMsgServerReceiver ${FILE_SRV_SRCS} ${FILE_SRV_HDRS})
target_link_libraries(eitOsMsgServerReceiver msgQServer timer node eitOsMsgServerSender)
target_include_directories(eitOsMsgServerReceiver PUBLIC 
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com/protocols/osMsg/osMsgServer/osMsgServerReceiver
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com/protocols/osMsg/osMsgServer/osMsgServerSender
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com/server)


