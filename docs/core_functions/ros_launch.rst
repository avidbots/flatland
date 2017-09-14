.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png

Launching Flatland Server Node
==============================

Flatland Server provides a launch file to start the 2D simulator. The parameters
for the launch file is specified below.

Run this following command to launch the simulator using default parameters

.. code-block:: bash

  $ roslaunch flatland_server server.launch world_path:=/path/to/world.yaml


Here are the full list of parameters with the default values 

.. code-block:: bash

  $ roslaunch flatland_server server.launch world_path:=/path/to/world.yaml \
                                            update_rate:=200.0 \
                                            step_size:=0.005 \
                                            show_viz:=true \
                                            viz_pub_rate:=30.0 \
                                            use_rviz:=false

* **world_path**: path to world.yaml
* **update_rate**: the real time rate to run the simulation loop in Hz
* **step_size**: amount of time to step each loop in seconds
* **show_viz**: show visualization, pops the flatland_viz window and publishes 
  visualization messages, either true or false
* **viz_pub_rate**: rate to publish visualization in Hz, works only when show_viz=true
* **use_rviz**:  works only when show_viz=true, set this to disable flatland_viz popup