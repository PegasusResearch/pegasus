
Pegasus Documentation
===================================

Overview
========

**Pegasus** is a quasi open-source Guidance, Navigation and Control (GNC) software package for controlling autonomous drones using ROS 2. 
It is designed to be used with the PX4 flight stack, but open enough to be used with other flight stacks as well. It is mainly written in C++ and Python.

This work is part of **Project Pegasus**, a semi-personal side project started by Marcelo Jacinto with the end-goal of supporting experimental validation of his Ph.D. It provides:

- A set of ROS 2 nodes with a modular state machine;

- A CAD model for a 3D printed drone used in the project;

- A set of Gazebo plugins to simulate the drone in Gazebo;

This project is provided as is, with no warranty or support. It is intended to be used as a reference for other researchers and developers in the field of control and robotics. It may NOT
be used for commercial purposes or military applications without the explicit consent from the project founder.

Developer Team
~~~~~~~~~~~~~~

- Project Founder
   - `Marcelo Jacinto <https://github.com/MarceloJacinto>`__, under the supervision of Prof. Rita Cunha (IST/ISR-Lisbon)
- Architecture
   - `Marcelo Jacinto <https://github.com/MarceloJacinto>`__
   - `João Pinto <https://github.com/jschpinto>`__
- Controllers
   - `Marcelo Jacinto <https://github.com/MarceloJacinto>`__
   - `João Pinto <https://github.com/jschpinto>`__
- CAD model
   - `Marcelo Jacinto <https://github.com/MarceloJacinto>`__
- Documentation
   - `Marcelo Jacinto <https://github.com/MarceloJacinto>`__
   - `João Pinto <https://github.com/jschpinto>`__

If you find ``Pegasus`` useful in your academic work, please cite the tech report below. It is also available (TODO).

.. code-block:: bibtex

   @misc{jacinto2024pegasus,
      title={Pegasus: A Quasi Open-Source Guidance, Control and Navigation Software Package for Autonomous Drones}, 
      author={Marcelo Jacinto and João Pinto and Rita Cunha},
      year={2024},
      eprint={},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
    }

Project Sponsors
================
The work developed by Marcelo Jacinto and João Pinto was supported by Ph.D. grants funded by Fundação para a Ciência e a Tecnologia (FCT).

.. raw:: html

   <p float="left" align="center">
      <img src="_static/logo_isr.png" width="200" align="center"/> 
      <img src="_static/larsys_logo.png" width="200" align="center"/> 
      <img src="_static/ist_logo.png" width="200" align="center"/> 
      <img src="_static/logo_fct.png" width="200" align="center"/> 
   </p>

As the author of this work, I would like to thank the support of the following coleagues and friends (in no particular order) during the development and testing of this project.

- `Pedro Trindade <https://scholar.google.com/citations?hl=pt-PT&user=eFG-wQ0AAAAJ>`__

- `João Pinto <https://github.com/jschpinto>`__

- `Gil Serrano <https://github.com/GilSerrano>`__

- `Jose Gomes <https://scholar.google.com/citations?hl=pt-PT&user=PECAagsAAAAJ>`__

- `Pedro Santos`

- `Joao Lehodey`

Not only have you made this project possible, but also crashing drones an enjoyable experience! Checkout their work and contributions in the field of control and robotics.

.. toctree::
   :maxdepth: 3
   :caption: Getting Started

   source/setup/installation
   source/setup/reference_frames

.. toctree::
   :maxdepth: 3
   :caption: Features

   source/features/autopilot
   source/features/modes
   source/features/controllers
   source/features/geofencing
   source/features/trajectory_manager
   source/console/terminal_console

.. toctree::
   :maxdepth: 3
   :caption: Simulation

   source/simulation/gazebo_classic
   source/simulation/pegasus_simulator

.. toctree::
   :maxdepth: 3
   :caption: Vehicles

   source/vehicles/pegasus
   source/vehicles/kopis
   source/vehicles/snapdragon

.. toctree::
   :maxdepth: 3
   :caption: Flight Arena

   source/arena/arena

.. toctree::
   :maxdepth: 1
   :caption: References

   source/references/license
   source/references/bibliography


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
