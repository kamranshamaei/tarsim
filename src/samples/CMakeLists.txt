add_subdirectory(clientSample)
#add_subdirectory(qt)

INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/samples DESTINATION ./user/client 
    PATTERN "samples/CMakeLists.txt" EXCLUDE)

INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/samples DESTINATION ./user/server 
    PATTERN "samples/CMakeLists.txt" EXCLUDE)

INSTALL(FILES ${CMAKE_SOURCE_DIR}/samples/README.md DESTINATION .)
INSTALL(FILES ${CMAKE_SOURCE_DIR}/samples/scara/win.txt DESTINATION ./default)