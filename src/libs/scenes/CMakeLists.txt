project (SceneBaseProj)


find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

include_directories(
    .
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc/protobufs
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/actorsRigidBody
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/configParser
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/kinematics
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com/server
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/node
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/object
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/gui
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/logging/logClient/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/collisionDetection
    ${VTK_INCLUDE_DIRS}
    )

set(FILE_GUI_HDRS 
    sceneBase.h
    sceneRobot.h
    sceneJointValues.h
    sceneEndEffectorFrame.h
    sceneButtons.h
    sceneFaults.h
    sceneSixDof.h
    sceneSpeed.h
    incCmdButton.h
    )
    
set(FILE_GUI_SRCS 
    sceneBase.cpp
    sceneRobot.cpp
    sceneJointValues.cpp
    sceneEndEffectorFrame.cpp
    sceneButtons.cpp
    sceneFaults.cpp
    sceneSixDof.cpp
    sceneSpeed.cpp
    incCmdButton.cpp
    )

add_library(sceneBase ${FILE_GUI_SRCS} ${FILE_GUI_HDRS})
target_link_libraries(sceneBase logClient ${VTK_LIBRARIES} node eitServer)
