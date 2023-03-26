#!/usr/bin/env bash

# -----------------------------------
# Install basic tools for development
# -----------------------------------
sudo apt-get update && \
     apt-get upgrade -y && \
     apt-get install -y --no-install-recommends \
     git \
     wget \
     python3-pip \
     && rm -rf /var/lib/apt/lists/*

# -----------------------------------------
# Install MAVSDK for interface with mavlink
# -----------------------------------------
git clone --recursive https://github.com/mavlink/MAVSDK.git && \
cd MAVSDK && \
cmake -Bbuild/default -DCMAKE_BUILD_TYPE=Release -H. && \
sudo cmake --build build/default --target install && \
cd .. && \
rm -r -f MAVSDK

# -----------------------------------------
# Install Mathematics Libraries
# -----------------------------------------

# Install Eigen version 3.4.0 (C++ equivalent of numpy in python):
wget -q https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz && \
tar xfpz eigen-3.4.0.tar.gz && \
cd eigen-3.4.0 && \
mkdir BUILD && \
cd BUILD && \
cmake .. && \
sudo make && \
sudo make install && \
cd .. && \
cd .. && \
sudo rm -R eigen-3.4.0 eigen-3.4.0.tar.gz