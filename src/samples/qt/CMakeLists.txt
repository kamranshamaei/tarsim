if(POLICY CMP0020)
  cmake_policy(SET CMP0020 NEW)
endif()

project (qtTarsim)

find_package(VTK REQUIRED)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
find_package(Qt5Widgets REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

file(GLOB UI_FILES *.ui)
file(GLOB QT_WRAP *.h)
file(GLOB CXX_FILES *.cpp)

qt5_wrap_ui(UISrcs ${UI_FILES})


add_executable(qtTarsim MACOSX_BUNDLE ${CXX_FILES} ${UISrcs} ${QT_WRAP})
qt5_use_modules(qtTarsim Core Gui)
target_link_libraries(qtTarsim tarsimLib ${VTK_LIBRARIES})
# vtk_module_autoinit is needed
vtk_module_autoinit(
    TARGETS qtTarsim
    MODULES ${VTK_LIBRARIES}
    )