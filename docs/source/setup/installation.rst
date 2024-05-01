Installation
============

.. image:: https://img.shields.io/badge/PX4--Autopilot-1.14.1-brightgreen.svg
   :target: https://github.com/PX4/PX4-Autopilot
   :alt: PX4-Autopilot 1.14.1

.. image:: https://img.shields.io/badge/Ubuntu-22.04LTS-brightgreen.svg
   :target: https://releases.ubuntu.com/22.04/
   :alt: Ubuntu 22.04

.. image:: https://img.shields.io/badge/ROS-Humble-brightgreen.svg
    :target: https://docs.ros.org/en/humble/index.html
    :alt: ROS 2 Humble

Start by installing ROS 2 Humble by following the instructions on the `ROS 2 Humble Documentation <https://docs.ros.org/en/humble/index.html>`__. Only after you have installed ROS 2, you can proceed with the installation of the Pegasus.

Installing External Dependencies
--------------------------------

1. Create a separate workspace for the Pegasus external dependencies, such that you only need to compile them once:

    .. code:: bash

        mkdir -p ~/pegasus_external
        cd ~/pegasus_external

2. Clone the `Pegasus External <https://github.com/PegasusResearch/pegasus_external>`__:

    .. code:: bash

        # Clone the repository
        git clone git@github.com:PegasusResearch/pegasus_external.git src

        # Compile the code 
        cd ..
        colcon build --symlink-install

3. Source the workspace in the .bashrc file:

    .. code:: bash

        echo "source ~/pegasus_external/install/setup.bash" >> ~/.bashrc


Installing Pegasus
------------------

1. Create a separate workspace that will contain the actual Pegasus code:

    .. code:: bash

        # Create the workspace
        mkdir -p ~/pegasus/src
        cd ~/pegasus_ws/src

        # Clone the repository
        git clone git@github.com:PegasusResearch/pegasus.git



In order to run gazebo clasic simulations with the Pegasus, you need to install the following dependencies:

Installing PX4-Autopilot
------------------------

In this first version of the Pegasus the PX4-Autopilot is used for handling motor control and sensor integration.
To install PX4-Autopilot, follow the following steps:

1. Install the dependencies (to be able to compile PX4-Autopilot):

    .. code:: bash

        # Linux packages
        sudo apt install git make cmake python3-pip
       
        # Python packages
        pip install kconfiglib jinja2 empy jsonschema pyros-genmsg packaging toml numpy future

2. Clone the `PX4-Autopilot <https://github.com/PX4/PX4-Autopilot>`__:

    .. code:: bash

        # Option 1: With HTTPS
        git clone https://github.com/PX4/PX4-Autopilot.git
        # Option 2: With SSH (you need to setup a github account with ssh keys)
        git clone git@github.com:PX4/PX4-Autopilot.git

3. Checkout to the stable version 1.14.1 and compile the code for software-in-the-loop (SITL) mode:

    .. code:: bash
        
        # Go to the PX4 directory
        cd PX4-Autopilot

        # Checkout to the latest stable release
        git checkout v1.14.1

        # Initiate all the submodules. Note this will download modules such as SITL-gazebo which we do not need
        # but this is the safest way to make sure that the PX4-Autopilot and its submodules are all checked out in 
        # a stable and well tested release
        git submodule update --init --recursive

        # Compile the code in SITL mode
        make px4_sitl_default gazebo

Installing Realsense
--------------------

