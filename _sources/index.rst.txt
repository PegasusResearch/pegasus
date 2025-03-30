
Pegasus Documentation
===================================

..  youtube:: BxUyFCwAXuo
   :width: 100%
   :align: center
   :privacy_mode:

Overview
========

**Pegasus** is a quasi open-source Guidance, Navigation and Control (GNC) software package for controlling autonomous drones using ROS 2. 
It is designed to be used with the PX4 flight stack, but open enough to be used with other flight stacks as well. It is mainly written in C++ and Python.

This work is part of **Project Pegasus**, a semi-personal side project started by Marcelo Jacinto with the end-goal of supporting experimental validation of his Ph.D. It provides:

- A set of ROS 2 nodes with a modular state machine;

- A CAD model for a 3D printed drone used in the project;

- An interface with PX4 and Pegasus Simulator and Gazebo Classic for 3D simulations;

This project is provided as is, with no warranty or support. It is intended to be used as a reference for other researchers and developers in the field of control and robotics. It may **NOT**
be used for commercial purposes or military applications without the explicit consent from the project founder.

Check the videos below to see the initial outdoor flight tests of the new Pegasus prototype:

  * First outdoor flight tests with camera pipeline:
  
    ..  youtube:: mLG3x38v5SY
      :width: 100%
      :align: center
      :privacy_mode:


  * Initial sensor calibration and outdoor control tests:
  
    ..  youtube:: dEQMgvO_WxI
      :width: 100%
      :align: center
      :privacy_mode:

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


While an article for the Pegasus GNC project is not publicly available yet, if you find ``Pegasus GNC`` useful in your academic work, please cite the "second most adequate paper":

.. code-block:: bibtex

   @INPROCEEDINGS{10556959,
      author={Jacinto, Marcelo and Pinto, João and Patrikar, Jay and Keller, John and Cunha, Rita and Scherer, Sebastian and Pascoal, António},
      booktitle={2024 International Conference on Unmanned Aircraft Systems (ICUAS)}, 
      title={Pegasus Simulator: An Isaac Sim Framework for Multiple Aerial Vehicles Simulation}, 
      year={2024},
      volume={},
      number={},
      pages={917-922},
      keywords={Simulation;Robot sensing systems;Real-time systems;Sensor systems;Sensors;Task analysis},
      doi={10.1109/ICUAS60882.2024.10556959}
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

- `Gil Serrano <https://scholar.google.com/citations?hl=pt-PT&user=h_6XghgAAAAJ>`__

- `Jose Gomes <https://scholar.google.com/citations?hl=pt-PT&user=PECAagsAAAAJ>`__

- `Pedro Santos <https://scholar.google.com/citations?hl=pt-PT&user=HOhIHJAAAAAJ>`__

- `Joao Lehodey`

Not only have you made this project possible, but also crashing drones an enjoyable experience! Checkout their work and contributions in the field of control and robotics. 

I also want to thank the support of:

- Manuel Rufino for assisting me in the soldering job of the drone, 
- José Tojeira for being an awesome safety pilot during the first flights and teach me the basics of drone piloting,
- Alexandre Lopes for being willing to help me during the flight tests of the Pegasus prototype,

and all the other friends, colleagues and family that have supported me during the development of this project.

.. toctree::
   :maxdepth: 5
   :caption: Getting Started

   source/setup/installation
   source/setup/project_setup
   source/setup/reference_frames

.. toctree::
   :maxdepth: 5
   :caption: Features

   source/features/autopilot/main
   source/console/terminal_console

.. toctree::
   :maxdepth: 5
   :caption: Simulation

   source/simulation/gazebo_classic
   source/simulation/pegasus_simulator

.. toctree::
   :maxdepth: 5
   :caption: Vehicles

   source/vehicles/vehicles

.. toctree::
   :maxdepth: 5
   :caption: Flight Arena

   source/arena/arena

.. toctree::
   :maxdepth: 5
   :caption: References

   source/references/license
   source/references/bibliography


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
