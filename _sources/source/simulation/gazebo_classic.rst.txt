Gazebo Classic
==============

Installing Gazebo Classic
-------------------------

1. Install gazebo classic:

    .. code:: bash

        # Make sure to uninstall gazebo garden (if installed)
        sudo apt remove gz-garden
        sudo apt install aptitude
        
        # Install gazebo (classic)
        sudo aptitude install gazebo libgazebo11 libgazebo-dev

        # Install the ROS packages for gazebo
        sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs


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

3. Checkout to the stable version 1.14.3 and compile the code for software-in-the-loop (SITL) mode:

    .. code:: bash
        
        # Go to the PX4 directory
        cd PX4-Autopilot

        # Checkout to the latest stable release
        git checkout v1.14.3

        # Compile the code in SITL mode
        make px4_sitl gazebo-classic

Setup the Pegasus Gazebo package
--------------------------------

