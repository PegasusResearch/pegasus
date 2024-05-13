# Pegasus

**Pegasus** is a quasi open-source Guidance, Navigation and Control (GNC) software package for controlling autonomous drones using ROS 2. 
It is designed to be used with the PX4 flight stack, but open enough to be used with other flight stacks as well. It is mainly written in C++ and Python.

This work is part of the **Pegasus Project**, a semi-personal side project started by Marcelo Jacinto with the end-goal of supporting experimental validation of his Ph.D. It provides:

- A set of ROS 2 nodes with a modular state machine;

- A CAD model for a 3D printed drone used in the project;

- A set of Gazebo plugins to simulate the drone in Gazebo;

This project is provided as is, with no warranty or support. It is intended to be used as a reference for other researchers and developers in the field of control and robotics. It may NOT
be used for commercial purposes or military applications without the explicit consent from the project founder.

If you find ``Pegasus`` useful in your academic work, please cite the tech report below. It is also available (TODO).

## Citation

If you find Pegasus Simulator useful in your academic work, please cite the paper below. It is also available [here](https://arxiv.org/abs/2307.05263).
```
@misc{jacinto2024pegasus,
      title={Pegasus: A Quasi Open-Source Guidance, Control and Navigation Software Package for Autonomous Drones}, 
      author={Marcelo Jacinto and João Pinto and Rita Cunha},
      year={2024},
      eprint={},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

## Main Developer Team

This simulation framework is an open-source effort, started by me, Marcelo Jacinto in January/2023. It is a tool that was created with the original purpose of serving my Ph.D. workplan for the next 4 years, which means that you can expect this repository to be mantained, hopefully at least until 2027.

* Project Founder
	* [Marcelo Jacinto](https://github.com/MarceloJacinto), under the supervision of <u>Prof. Rita Cunha</u> (IST/ISR-Lisbon)
* Architecture
  * [Marcelo Jacinto](https://github.com/MarceloJacinto)
  * [João Pinto](https://github.com/jschpinto)
* CAD Model
  * [Marcelo Jacinto](https://github.com/MarceloJacinto)
* Documentation
  * [Marcelo Jacinto](https://github.com/MarceloJacinto)
  * [João Pinto](https://github.com/jschpinto)
  * [Pedro Trindade](https://scholar.google.com/citations?hl=pt-PT&user=eFG-wQ0AAAAJ)

As the author of this work, I would like to thank the support of the following coleagues and friends (in no particular order) during the development and testing of this project.
[Pedro Trindade](https://scholar.google.com/citations?hl=pt-PT&user=eFG-wQ0AAAAJ), [João Pinto](https://github.com/jschpinto), [Gil Serrano](https://github.com/GilSerrano), [Jose Gomes](https://scholar.google.com/citations?hl=pt-PT&user=PECAagsAAAAJ), Pedro Santos and Joao Lehodey. Not only have you made this project possible, but also crashing drones an enjoyable experience! Checkout their work and contributions in the field of control and robotics.

 
## Project Sponsors
The work developed by Marcelo Jacinto and João Pinto was supported by Ph.D. grants funded by Fundação para a Ciência e Tecnologia (FCT).

<p float="left" align="center">
  <img src="docs/_static/logo_isr.png" width="200" align="center"/> 
  <img src="docs/_static/larsys_logo.png" width="200" align="center"/> 
  <img src="docs/_static/ist_logo.png" width="200" align="center"/> 
  <img src="docs/_static/logo_fct.png" width="200" align="center"/> 
</p>
