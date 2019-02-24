project (TarsimClientInterfaceProj)

add_subdirectory(exposed)

# Find python
execute_process(
  COMMAND "which" "python"
  OUTPUT_VARIABLE DEFAULT_PYTHON)
       
message("Current python: ${DEFAULT_PYTHON}")
    
string(REPLACE "/bin/python\n" ""
    DEFAULT_PYTHON_DIRECTORY
    ${DEFAULT_PYTHON})
    
SET(PYTHON_INCLUDE_DIRS "${DEFAULT_PYTHON_DIRECTORY}/include/python${PYTHON_VERSION}/")
SET(DEFAULT_PYTHON_INCLUDE_DIRS "${DEFAULT_PYTHON_DIRECTORY}/include/python${PYTHON_VERSION}/")
SET(PYTHON_LIBRARY      "${DEFAULT_PYTHON_DIRECTORY}/lib/libpython${PYTHON_VERSION}.so")


find_package(PythonLibs REQUIRED)

include_directories(${PYTHON_INCLUDE_DIRS})

include_directories(
    .
    ${TARSIM_LIBRARIES_SOURCE_DIRECTORY}/com/client
    /usr/include/python2.7
    )

set(FILE_INTERFACE_HDRS
    )
    
set(FILE_INTERFACE_SRCS 
    tarsimClientInterface.cpp
    )

add_library(tarsimClientInterface ${FILE_INTERFACE_SRCS} 
    ${FILE_INTERFACE_HDRS} ${FILE_INTERFACE_EXTRA})
target_link_libraries(tarsimClientInterface tarsimClientExposed)


# Files to install with Python
set(PYTHON_INSTALL_FILES
    ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/tarsimClient.so
    ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/tarsimClientInterface.so
    ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/tarsimClientExposed.so
    ${CMAKE_BINARY_DIR}/wrappers/python/tarsimClient.py)
    
# Configure setup.py and copy to output directory
set(SETUP_PY_IN ${CMAKE_CURRENT_SOURCE_DIR}/tarsimClient.py)
set(SETUP_PY_OUT ${CMAKE_BINARY_DIR}/wrappers/python/tarsimClient.py)
configure_file(${SETUP_PY_IN} ${SETUP_PY_OUT})

set(SETUP_PY_IN ${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in)
set(SETUP_PY_OUT ${CMAKE_BINARY_DIR}/wrappers/python/setup.py)
configure_file(${SETUP_PY_IN} ${SETUP_PY_OUT})

# Install target to call setup.py
add_custom_target(install-python
    DEPENDS ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/tarsimClientInterface.so
    COMMAND python ${SETUP_PY_OUT} install)
    
    
