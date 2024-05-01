Pegasus Simulator
=================

To use the Pegasus GNC code with the Pegasus Simulator, start by following the installation instructions in :ref:`Installation` page. After this step:

1. Follow the installation instructions for the Pegasus Simulator in the
`Pegasus Simulator Documentation <https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html>`__. An NVIDIA RTX graphics card is required to run the simulator.

2. On the Pegasus workspace, clone the following repository:

    .. code-block:: bash

        # Go to the src folder of the Pegasus workspace
        cd ~/pegasus_ws/src

        # Clone the repository that contains the launch files to launch both the Pegasus GNC code and the Pegasus Simulator in the same environment
        git clone git@github.com:PegasusResearch/pegasus_isaac.git

        # Compile the code
        cd ~/pegasus_ws
        colcon build --symlink-install

        # Source the workspace
        source ~/pegasus_ws/install/setup.bash

3. To run an experiment:

    .. code-block:: bash

        # Launch the Pegasus GNC code and the Pegasus Simulator
        ros2 launch pegasus_isaac isaac.launch.py