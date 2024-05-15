Autopilot
=========

At the center of the Guidance and Control system is the Autopilot, implemented in C++. This is located inside the ``pegasus_autopilot/autopilot`` package. 
The Autopilot is responsible for keeping track of the current operating ``state`` of the vehicle. It exposes the ``control`` API 


taking in the current state of the rocket and the desired state of the rocket and outputting the control commands to the rocket. The Autopilot is responsible for the following tasks:




.. code:: C++

    #include <test.cpp>