FROM ubuntu:20.04

ENV DEBIAN_FRONTEND noninteractive

USER root

RUN apt -y update && apt -y upgrade &&\
    apt -y install git gdb vim build-essential sudo

# Generic Libs ---------------------------------------------------------------------------------------------------------
RUN apt -y update && \
    apt install -y --no-install-recommends cmake cmake-gui libeigen3-dev autoconf automake libtool curl make g++ unzip libglew-dev

WORKDIR /workdir

# Protobuf -------------------------------------------------------------------------------------------------------------
RUN git clone https://github.com/protocolbuffers/protobuf.git &&\
    cd protobuf &&\
    git checkout v3.14.0 &&\
    git submodule update --init --recursive &&\
    ./autogen.sh &&\
    ./configure &&\
    make -j `nproc` &&\
    make install &&\
    ldconfig

# VTK ------------------------------------------------------------------------------------------------------------------
ENV VTK_COMMIT 5e2ca5f054de37638e7473148fddebf9a40f8c98
RUN git clone https://gitlab.kitware.com/vtk/vtk.git VTK &&\
    cd VTK && git reset --hard ${VTK_COMMIT} &&\
    mkdir VTK-build &&\
    cd VTK-build &&\
    cmake \
        -DCMAKE_BUILD_TYPE:STRING=Release \
        -DVTK_Group_Qt:BOOL=ON \
        -DBUILD_SHARED_LIBS:BOOL=ON \
        -DVTK_QT_VERSION:STRING=5 \
        -DQT_QMAKE_EXECUTABLE:PATH=/usr/bin/qmake \
        .. &&\
    make -j $(nproc) && \
    make install &&\
    ldconfig

# TARSIM ---------------------------------------------------------------------------------------------------------------
RUN git clone https://github.com/kamranshamaei/tarsim.git tarsim &&\
    cd tarsim &&\
    mkdir tarsim-build &&\
    cd tarsim-build &&\
    cmake ../src &&\
    make -j $(nproc) && \
    make install &&\
    sudo mv ./tarsim /opt &&\
    ln -s /opt/tarsim/tarsim /usr/local/bin/tarsim &&\
    ln -s /opt/tarsim/user/client/inc/ /usr/local/include/tarsim


# Enviornment Setup ----------------------------------------------------------------------------------------------------
ARG UNAME="tarsim"

# User and password
RUN adduser --disabled-password --shell /bin/bash --gecos "User" ${UNAME} && \
    usermod -aG sudo ${UNAME} && \
    echo "${UNAME} ALL = (ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudoers_${UNAME}

USER ${UNAME}
WORKDIR /home/${UNAME}

# Last line ------------------------------------------------------------------------------------------------------------
