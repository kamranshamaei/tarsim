add_subdirectory(protobufs)

configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/cadOff.png 
    ${CMAKE_BINARY_DIR}/bin/icons/cadOff.png COPYONLY)
    
configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/cadOn.png 
    ${CMAKE_BINARY_DIR}/bin/icons/cadOn.png COPYONLY)

configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/linesOff.png 
    ${CMAKE_BINARY_DIR}/bin/icons/linesOff.png COPYONLY)
    
configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/linesOn.png 
    ${CMAKE_BINARY_DIR}/bin/icons/linesOn.png COPYONLY)

configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/pointsOff.png 
    ${CMAKE_BINARY_DIR}/bin/icons/pointsOff.png COPYONLY)
    
configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/pointsOn.png 
    ${CMAKE_BINARY_DIR}/bin/icons/pointsOn.png COPYONLY)

configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/pathOff.png 
    ${CMAKE_BINARY_DIR}/bin/icons/pathOff.png COPYONLY)
    
configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/pathOn.png 
    ${CMAKE_BINARY_DIR}/bin/icons/pathOn.png COPYONLY)

configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/recordOff.png 
    ${CMAKE_BINARY_DIR}/bin/icons/recordOff.png COPYONLY)
    
configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/recordOn.png 
    ${CMAKE_BINARY_DIR}/bin/icons/recordOn.png COPYONLY)

configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/framesOff.png 
    ${CMAKE_BINARY_DIR}/bin/icons/framesOff.png COPYONLY)
    
configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/framesOn.png 
    ${CMAKE_BINARY_DIR}/bin/icons/framesOn.png COPYONLY)

configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/planesOff.png 
    ${CMAKE_BINARY_DIR}/bin/icons/planesOff.png COPYONLY)
    
configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/planesOn.png 
    ${CMAKE_BINARY_DIR}/bin/icons/planesOn.png COPYONLY)

configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/view.png 
    ${CMAKE_BINARY_DIR}/bin/icons/view.png COPYONLY)
    
configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/exit.png 
    ${CMAKE_BINARY_DIR}/bin/icons/exit.png COPYONLY)

configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/exit.png 
    ${CMAKE_BINARY_DIR}/bin/icons/exit.png COPYONLY)

configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/sixDof.png 
    ${CMAKE_BINARY_DIR}/bin/icons/sixDof.png COPYONLY)

configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/dofButton.png 
    ${CMAKE_BINARY_DIR}/bin/icons/dofButton.png COPYONLY)

configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/inc.png 
    ${CMAKE_BINARY_DIR}/bin/icons/inc.png COPYONLY)

configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/dec.png 
    ${CMAKE_BINARY_DIR}/bin/icons/dec.png COPYONLY)

configure_file(
    ${CMAKE_SOURCE_DIR}/config/icons/README.md 
    ${CMAKE_BINARY_DIR}/bin/icons/README.md COPYONLY)

INSTALL(DIRECTORY ${CMAKE_BINARY_DIR}/bin/icons/ DESTINATION ./icons/)
INSTALL(DIRECTORY ${CMAKE_BINARY_DIR}/bin/icons/ DESTINATION ./user/server/icons/)
