# Tarsim

![alt text](https://raw.githubusercontent.com/kamranshamaei/tarsim/development/doc/pics/fanuc.png)
![alt text](https://raw.githubusercontent.com/kamranshamaei/tarsim/development/doc/pics/kuka.png)
![alt text](https://raw.githubusercontent.com/kamranshamaei/tarsim/development/doc/pics/scara.png)

# Install
## Install VTK
Download the latest VTK version (e.g. 8.2.0): https://vtk.org/download/#latest

```
cd /tmp
mkdir VTK-build
cd VTK-build
cmake -DVTK_QT_VERSION:STRING=5 \
      -DQT_QMAKE_EXECUTABLE:PATH=/path/to/qt5.2.1-install/5.2.1/gcc_64/bin/qmake \
      -DVTK_Group_Qt:BOOL=ON \
      -DCMAKE_PREFIX_PATH:PATH=/path/to/qt.5.2.1-install/5.2.1/gcc_64/lib/cmake  \
      -DBUILD_SHARED_LIBS:BOOL=ON\
      /path/to/VTK-8.2.0

make -j8
sudo make install
```

## Install Protobuf
Install protobuf version 3 (e.g. v3.11.2):

```
cd /tmp
git clone https://github.com/protocolbuffers/protobuf.git
git checkout v3.11.2
cd protobuf
git submodule update --init --recursive
./autogen.sh
./configure
make -j8
make check
sudo make install
sudo ldconfig
```

## Build and Install Tarsim
Clone, build, and install tarsim as:

```
cd /tmp
git clone https://github.com/kamranshamaei/tarsim.git
mkdir -p /path/to/build
Run cmake-gui and point build to the /path/to/build and source to /tmp/tarsim/src. Click on configure and generated twice.
cd /path/to/build
make -j8
make install
```
At this point, you'd have a tarsim folder in your build folder. Copy and paste it to a desired location as /path/to/tarsim.
```
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/tarsim/lib' >> ~/.bashrc
source ~/.bashrc
```

# Run
To run tarsim:

```
/path/to/tarsim/tarsim -c /path/to/robot/rbs.txt
``` 
