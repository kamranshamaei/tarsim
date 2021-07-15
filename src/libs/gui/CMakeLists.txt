project (GuiProj)


find_package(VTK REQUIRED)

include_directories(
    .
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc/protobufs
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/actorsRigidBody
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/node
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/object
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/scenes
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/kinematics
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/configParser
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com/server
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/logging/logClient/inc
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/collisionDetection
    ${BUILD_INCLUDE_OUTPUT_DIRECTORY}
    ${VTK_INCLUDE_DIRS}
    )

set(FILE_GUI_HDRS 
    gui.h
    )
    
set(FILE_GUI_SRCS 
    gui.cpp
    )
    
add_library(gui ${FILE_GUI_SRCS} ${FILE_GUI_HDRS})
target_link_libraries(gui logClient ${VTK_LIBRARIES} node sceneBase 
    kinematics configParser)
# vtk_module_autoinit is needed
vtk_module_autoinit(
    TARGETS gui
    MODULES ${VTK_LIBRARIES}
    )