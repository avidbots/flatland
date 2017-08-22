Flatland ROS Services
=====================
Flatland offers means for manipulating the simulation through ROS services. 
The following are the services offered by the simulator. 

Spawning Models
---------------
Models can be spawned dynamically after instantiation of the world. The parameters
are same as the ones for models in YAML files, all input parameters are required.

Request:

.. code-block:: bash

  string yaml_path           # path to model yaml file
  string name                # name of the model
  string ns                  # namespace, use "" for no namespace
  geometry_msgs/Pose2D pose  # model pose 
  
Response:

.. code-block:: bash

  bool success    # to check if the operation is successful
  string message  # error message if unsuccessful


Deleting Models
---------------

Request:

.. code-block:: bash

  string name  # name of the model to delete
  
Response:

.. code-block:: bash

  bool success    # check if the operation is successful
  string message  # error message if unsuccessful


