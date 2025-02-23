Mode Interface
==============

The interface for the autopilot operating modes is defined in the autopilot package under the ``pegasus_autopilot/autopilot/include/autopilot/mode.hpp`` header file.
The ``Mode`` class is an abstract class that defines the interface for the autopilot operating modes. 

The autopilot operating modes must inherit from the ``Mode`` class and implement the methods
highlighted in yellow (lines 42-47) in the code snippet below:

.. literalinclude:: ../../../../../pegasus_autopilot/autopilot/include/autopilot/mode.hpp
   :language: c++
   :emphasize-lines: 42-47
   :lines: 62-130
   :lineno-start: 1

Additionally, the ``Mode`` class provides methods to get:

1. The current ``State`` of the vehicle can be accessed by calling ``get_vehicle_state()`` . It returns the following struct:

.. literalinclude:: ../../../../../pegasus_autopilot/autopilot/include/autopilot/state.hpp
   :language: c++
   :lines: 52,54-60,77
   :lineno-start: 1

2. The ``Status`` of the vehicle can be accessed by calling ``get_vehicle_status()`` . It returns the following struct:  

.. literalinclude:: ../../../../../pegasus_autopilot/autopilot/include/autopilot/state.hpp
   :language: c++
   :lines: 52,62-67,77
   :lineno-start: 1

3. The ``Constants`` such as mass or thrust curve can be accessed by calling ``get_vehicle_constants`` . It returns the following struct:

.. literalinclude:: ../../../../../pegasus_autopilot/autopilot/include/autopilot/state.hpp
   :language: c++
   :lines: 52,69-75,77
   :lineno-start: 1

This means that to access these values, you do not need to manually subscribe to the ROS 2 topics. The autopilot
takes care of that for you. However, if you are required to access other values that are not provided by the ``Mode`` class, or if you need to publish
values to the ROS 2 topics, you can do so by using the ``node_`` shared pointer that is a member of the ``Mode`` class.

4. The ``Mode`` class also provides a shared pointer to a ``controller_`` object that can be used to access the controller interface. To check
the available methods in the controller interface, please refer to the :ref:`Controllers` page. This is useful if you want to send control commands to the vehicle
without having to publish to the ROS 2 topics directly. The autopilot takes care of that for you.

5. The ``Mode`` class also provides a shared pointer to a ``trajectory_manager_`` object that can be used to access the trajectory manager interface. To check
the available methods in the trajectory manager interface, please refer to the :ref:`Trajectory Manager` page. This is useful if your mode needs to follow a trajectory
and you don't want to manually subscribe to the ROS 2 topics or hard-code the trajectory in the mode.