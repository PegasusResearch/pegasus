Gazebo Classic
==============

In this section, we will explain how to install and setup Gazebo Classic and PX4-Autopilot for performing simulations with pegasus.
This was tested on Ubuntu 22.04.

.. image:: https://img.shields.io/badge/PX4--Autopilot-1.14.3-brightgreen.svg
   :target: https://github.com/PX4/PX4-Autopilot
   :alt: PX4-Autopilot 1.14.3

.. image:: https://img.shields.io/badge/Ubuntu-22.04LTS-brightgreen.svg
   :target: https://releases.ubuntu.com/22.04/
   :alt: Ubuntu 22.04

.. image:: https://img.shields.io/badge/ROS-Humble-brightgreen.svg
    :target: https://docs.ros.org/en/humble/index.html
    :alt: ROS 2 Humble

.. image:: https://img.shields.io/badge/Gazebo-11%20(classic)-brightgreen.svg
    :target: https://classic.gazebosim.org/
    :alt: Gazebo 11 (Classic)

Installing Gazebo Classic
-------------------------

Start by installing gazebo classic. Make sure that you do not have the `gz-garden` package installed, as it may conflict with the installation of gazebo classic.

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

        # GStreamer (for video streaming)
        sudo apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly -y

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

4. Add the following line to your .bashrc file:

    .. code:: bash

        echo "export PX4_DIR=$(pwd)" >> ~/.bashrc

    .. admonition:: Note

        Adding this line to the .bashrc file is important as the Pegasus Gazebo package will need to know the location of the PX4-Autopilot directory, and the launch files
        will use this environment variable to find the necessary files.

Installing the Pegasus Gazebo package
-------------------------------------

1. In the Pegasus workspace, clone the following repository:

    .. code-block:: bash

        # Go to the src folder of the Pegasus workspace
        cd ~/pegasus/src

        # Clone the repository (SSH)
        git clone git@github.com:PegasusResearch/pegasus_gazebo.git

2. Compile the code:

    .. code:: bash

        # Go to the workspace
        cd ~/pegasus

        # Compile the code
        colcon build --symlink-install

3. Source the workspace in the .bashrc file:

    .. code:: bash

        echo "source ~/pegasus/install/setup.bash" >> ~/.bashrc

Running a Simulation
--------------------

..  youtube:: exatJ6hvD1Q
    :width: 100%
    :align: center
    :privacy_mode:

To run a simulation with the Pegasus Gazebo package, follow the steps below:

1. Start the gazebo server with a world file:

    .. code:: bash

        ros2 launch pegasus_gazebo taguspark.launch.py

2. On another terminal, start a vehicle simulation:

    .. code:: bash

        ros2 launch pegasus_gazebo iris.launch.py vehicle_id:=<vehicle_id>

    You should replace the ``<vehicle_id>`` with the vehicle id you want to connect to. If no option is provided, the vehicle will run with ID 1.

3. On a third terminal, start the pegasus console to operate the vehicle:

    .. code:: bash

        ros2 run pegasus_console pegasus_console

    .. admonition:: Pegasus Console

        Check the :ref:`Terminal console` page for more information on how to operate the vehicle.


**World launch files available for simulation:**

- ``taguspark.launch.py``: A simple world with a "box" the size of the Taguspark Mocap Arena.
- ``empty.launch.py``: An empty world with no obstacles (for low-weight simulations).

**Vehicle launch files available for simulation:**

- ``iris.launch.py``: A simple Iris drone.
- ``iris_fpv.launch.py``: A simple Iris drone with a front-facing monocular camera.