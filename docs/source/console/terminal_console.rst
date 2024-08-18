Terminal console
================


.. image:: /_static/terminal_console/terminal_console.png
    :width: 700px
    :align: center
    :alt: Terminal Console

To execute the terminal console you need to run the following command:

.. code:: bash

    ros2 run pegasus_console pegasus_console -i <vehicle_id> -n <namespace>

You should replace the ``<vehicle_id>`` with the vehicle id you want to connect to. If not option is provided, the console will attempt to connect to ID 1.
If you want to connect to a specific namespace, you should replace the ``<namespace>`` with the desired namespace. If no namespace is provided, the console will connect to the default namespace "drone".