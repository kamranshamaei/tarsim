#!/bin/bash
export PACKAGE_FOLDER=$(pwd)

# Create a temporary installation directory
export TEMP_FOLDER=/tmp/install_eit
mkdir "$TEMP_FOLDER"

# Build and install VTK
mkdir "$TEMP_FOLDER/vtk-build"

( cd "$TEMP_FOLDER/vtk-build" && cmake -DCMAKE_BUILD_TYPE:STRING=Release -DVTK_Group_Qt:BOOL=false "$PACKAGE_FOLDER/vtk/8.1.0/src")

( cd "$TEMP_FOLDER/vtk-build" && sudo make -j8 install)

rm -rf "$TEMP_FOLDER"

echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/tarsim/lib:/usr/lib/include:/usr/local/lib' >> ~/.bashrc

source ~/.bashrc
