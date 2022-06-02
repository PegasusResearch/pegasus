#!/usr/bin/env bash

# -----------------------------------
# Install basic tools for development
# -----------------------------------
sudo apt-get update && \
     apt-get upgrade -y && \
     apt-get install -y --no-install-recommends \
     git \
     wget \
     pip3 \
     && rm -rf /var/lib/apt/lists/*


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

# -------------------------------------------------
# Install the documentation generation requrirement
# -------------------------------------------------
sudo apt-get update && \
     apt-get upgrade -y && \
     apt-get install -y --no-install-recommends \
     doxygen \
     && rm -rf /var/lib/apt/lists/*

pip3 install --no-cache-dir \
    mkdocs \
    mkdocs-material \
    mkdocs-bibtex \
    mkdocs-git-revision-date-plugin \
    mkdocs-git-revision-date-localized-plugin \
    mkdocs-monorepo-plugin \
    mkdocs-macros-plugin \
    mkdocs-include-markdown-plugin \
    mkdocs-git-revision-date-localized-plugin \
    mkdocs-include-dir-to-nav \
    mike \
    ruamel.yaml

mkdir doxybook2 && \
    cd doxybook2 && \
    wget https://github.com/matusnovak/doxybook2/releases/download/v1.4.0/doxybook2-linux-amd64-v1.4.0.zip && \
    unzip doxybook2-linux-amd64-v1.4.0.zip && \
    cd bin && mv doxybook2 /usr/bin/ && \
    cd ${HOME} && rm -f -r doxybook2