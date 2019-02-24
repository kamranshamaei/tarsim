add_subdirectory(protobufs)

INSTALL(FILES ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/inc/simulatorMessages.h 
    DESTINATION ./user/client/inc)

if (NOT DEFINED TARSIM_VERSION)
    SET(TARSIM_VERSION "V1.0" CACHE STRING "Choose the version" FORCE)
    set_property(CACHE TARSIM_VERSION PROPERTY STRINGS "V1.0")
endif()

if (NOT DEFINED TARSIM_LICENSE_TYPE)
    SET(TARSIM_LICENSE_TYPE "Academic" CACHE STRING "Choose the type of license" FORCE)
    set_property(CACHE TARSIM_LICENSE_TYPE PROPERTY STRINGS "Academic" "Professional")
endif()

if (NOT DEFINED TARSIM_LICENSE_OWNER_NAME)
    SET(TARSIM_LICENSE_OWNER_NAME "Kamran Shamaei" CACHE STRING "Provide the name of the owner" FORCE)
    set_property(CACHE TARSIM_LICENSE_OWNER_NAME PROPERTY STRINGS "Kamran Shamaei")
endif()

configure_file(cmake_defs.h.in ${BUILD_INCLUDE_OUTPUT_DIRECTORY}/cmake_defs.h)
