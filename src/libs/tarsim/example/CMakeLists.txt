cmake_minimum_required (VERSION 2.6)

project (example)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")

find_package(VTK REQUIRED)

include_directories (
    path/to/user/server/inc
    ${VTK_DIR}
    ${VTK_INCLUDE_DIRS}
    )

add_library(tarsimLib SHARED IMPORTED)
set_property(TARGET tarsimLib PROPERTY IMPORTED_LOCATION "path/to/user/server/lib")

add_executable(useTarsim tarsim.cpp)
target_link_libraries(useTarsim tarsimLib ${VTK_LIBRARIES})

# vtk_module_autoinit is needed
vtk_module_autoinit(
    TARGETS useTarsim
    MODULES ${VTK_LIBRARIES}
    )