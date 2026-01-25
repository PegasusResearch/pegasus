Gazebo Jetty
============

In this section, we will explain how to install and setup Gazebo Jetty and PX4-Autopilot for performing simulations with pegasus.
This was tested on Ubuntu 24.04.

.. image:: https://img.shields.io/badge/PX4--Autopilot-1.14.3-brightgreen.svg
   :target: https://github.com/PX4/PX4-Autopilot
   :alt: PX4-Autopilot 1.14.3

.. image:: https://img.shields.io/badge/Ubuntu-24.04LTS-brightgreen.svg
   :target: https://releases.ubuntu.com/24.04/
   :alt: Ubuntu 24.04

.. image:: https://img.shields.io/badge/ROS-Jazzy-brightgreen.svg
    :target: https://docs.ros.org/en/jazzy/index.html
    :alt: ROS 2 Jazzy

.. image:: https://img.shields.io/badge/Gazebo-Jetty-brightgreen.svg
    :target: https://www.gazebosim.org/
    :alt: Gazebo Jetty

Installing Gazebo Jetty
-------------------------

Follow the installation steps provided in the official Gazebo Jetty documentation: `Gazebo Jetty Installation Guide <https://gazebosim.org/docs/jetty/install_ubuntu/>`__, which are provided here for convenience:

    .. code:: bash

        # Install the necessary tools
        sudo apt-get update
        sudo apt-get install curl lsb-release gnupg
        
        # Install gazebo (jetty)
        sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
        sudo apt-get update
        sudo apt-get install gz-jetty

        # Make sure multicast is enabled on your network interface
        sudo ufw allow in proto udp to 224.0.0.0/4
        sudo ufw allow in proto udp from 224.0.0.0/4  

        # Install ROS 2 Jazzy packages for Gazebo
        sudo apt install ros-jazzy-ros-gz  
        sudo ldconfig


Installing PX4-Autopilot
------------------------

In this first version of the Pegasus the PX4-Autopilot is used for handling motor control and sensor integration.
To install PX4-Autopilot, follow the following steps:

1. Install the dependencies (to be able to compile PX4-Autopilot):

    .. code:: bash

        # Linux packages
        sudo apt install git make cmake python3-pip
       
        # Python packages
        pip install kconfiglib jinja2 empy==3.3.4 jsonschema pyros-genmsg packaging toml numpy future

        # GStreamer (for video streaming)
        sudo apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly -y

2. Clone the `PX4-Autopilot <https://github.com/PX4/PX4-Autopilot>`__:

    .. code:: bash

        # Option 1: With HTTPS
        git clone https://github.com/PX4/PX4-Autopilot.git
        # Option 2: With SSH (you need to setup a github account with ssh keys)
        git clone git@github.com:PX4/PX4-Autopilot.git

3. Checkout to the stable version 1.16.1 and compile the code for software-in-the-loop (SITL) mode:

    .. code:: bash
        
        # Go to the PX4 directory
        cd PX4-Autopilot

        # Checkout to the latest stable release
        git checkout v1.16.1

        # Export the compilers (important if using python conda environments)
        export CXX=g++
        export CC=gcc

        # Compile the code in SITL mode
        make px4_sitl_default

4. Add the following line to your .bashrc file:

    .. code:: bash

        echo "export PX4_DIR=$(pwd)" >> ~/.bashrc

    .. admonition:: Note

        Adding this line to the .bashrc file is important as the Pegasus Gazebo package will need to know the location of the PX4-Autopilot directory, and the launch files
        will use this environment variable to find the necessary files.