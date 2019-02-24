#!/bin/bash
export PACKAGE_FOLDER=$(pwd)

# Create a temporary installation directory
export TEMP_FOLDER=/tmp/install_eit
mkdir "$TEMP_FOLDER"

# Build and install protobuf
mkdir "$TEMP_FOLDER/protobuf-build"

( cd "$TEMP_FOLDER/protobuf-build" && cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE:STRING=Release -Dprotobuf_BUILD_TESTS:BOOL=false -Dprotobuf_BUILD_SHARED_LIBS:BOOL=true "$PACKAGE_FOLDER/protobuf-master/cmake")

( cd "$TEMP_FOLDER/protobuf-build" && make -j8 && sudo make -j8 install && sudo ldconfig)

# Build and install VTK
mkdir "$TEMP_FOLDER/vtk-build"

( cd "$TEMP_FOLDER/vtk-build" && cmake -DCMAKE_BUILD_TYPE:STRING=Release -DVTK_Group_Qt:BOOL=true "$PACKAGE_FOLDER/vtk/8.1.0/src")

( cd "$TEMP_FOLDER/vtk-build" && sudo make -j8 install)

rm -r "$TEMP_FOLDER"

echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/tarsim/lib' >> ~/.bashrc

source ~/.bashrc
