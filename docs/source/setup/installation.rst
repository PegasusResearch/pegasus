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

2. Compile the code:

    .. code:: bash

        # Go to the workspace
        cd ~/pegasus

        # Compile the code
        colcon build --symlink-install

3. Source the workspace in the .bashrc file:

    .. code:: bash

        echo "source ~/pegasus/install/setup.bash" >> ~/.bashrc

Running Simulations with Pegasus
--------------------------------
- In order to run gazebo clasic simulations with the Pegasus, proceed to the page :ref:`Gazebo Classic`.

- To run simulations with the Pegasus Simulator (Isaac Sim), proceed to the page :ref:`Pegasus Simulator`.

Installing Realsense
--------------------

