#nvidia drivier support
FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 as nvidia
LABEL description="nvidia driver for OpenGL"
##################################################
FROM ubuntu:16.04 as builder
LABEL description="ubuntu"

#essential gl dynamic libraries
COPY --from=nvidia /usr/local /usr/local
COPY --from=nvidia /etc/ld.so.conf.d/glvnd.conf /etc/ld.so.conf.d/glvnd.conf

#some fixed environment variables
ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all
ENV CMAKE_PREFIX_PATH=/usr/local/lib64
ENV LD_LIBRARY_PATH=/usr/local/lib64

#basic dependencies.
#compile dependencies
#software-properties-common for add-apt-repository
#ca-certificates for verification
RUN apt-get update && apt-get install -y \
    software-properties-common \
    ca-certificates \
    build-essential \
    mesa-utils \
    glmark2 \
    cmake \
    sudo \
    vim \
    git \
    tar \
    unzip \
    wget \
    curl

#gcc8
RUN add-apt-repository -y ppa:jonathonf/gcc && \
    apt-get update && \
    apt-get install -y gcc-8 g++-8 && \
    rm -rf /usr/bin/gcc /usr/bin/g++ && \
    ln -s /usr/bin/g++-8 /usr/bin/g++ && \
    ln -s /usr/bin/gcc-8 /usr/bin/gcc

#boost pcl protobuf qt5.5
RUN apt-get update && \
    apt-get install -y \
    libboost-dev \
    libpcl-dev \
    libprotobuf-dev \
    protobuf-compiler \
    qt5-default \
    qttools5-dev-tools \
    libqt5opengl5-dev

#3rd party
WORKDIR /home/zhihui/library

#osg 3.7
RUN git clone https://github.com/openscenegraph/OpenSceneGraph.git && \
    cd OpenSceneGraph && mkdir build && cd build \
    && cmake .. && make -j6 && make install

#osgearth 2.10
RUN git clone https://github.com/gwaldron/osgearth.git && \
    cd osgearth && mkdir build && cd build \
    && cmake -DOSGEARTH_USE_QT=ON .. && make -j6 && make install

#draco 1.3.5
RUN git clone https://github.com/google/draco.git && \
    cd draco && mkdir build && cd build \
    && cmake .. && make -j6 && make install

#muduo
RUN git clone https://github.com/chenshuo/muduo.git && \
    cd muduo && mkdir build && cd build \
    && cmake .. && make -j6 && make install

#self
WORKDIR /home/zhihui/workspace

RUN git clone https://github.com/Wkkkkk/SLAMMapping.git && \
    cd SLAMMapping && mkdir build && cd build \
    && cmake .. && make -j6

EXPOSE 2000

CMD /home/zhihui/workspace/SLAMMapping/bin/client