.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png

Writing Model Plugins
=====================
Flatland is designed with extensibility in mind. Users can write their own Plugins
to tailor the simulator for their specific needs. A Model Plugin is a plugin
that is attached to a given model and perform specific functions for
that model. You can check flatland_plugins for existing examples of plugins.


ModelPlugin Base Class Overview
-------------------------------

The plugin system is implemented using `ROS pluginlib <http://wiki.ros.org/pluginlib>`_.
All model plugins must implement the base class ModelPlugin with the critical 
class members listed below. When the model plugin initializes, the YAML Node
containing the parameters for the plugin is passed in. It is up to the plugin to
extract data from the YAML Nodes and throw exceptions if any errors occur. Any
exceptions derived from std::exception will be caught and handled gracefully. 
However, we recommend throwing exceptions from flatland_server/exceptions.h.

.. code-block:: Cpp

  class ModelPlugin {
   public:

    // ROS node handle initialize with robot namespace, if namespace is "", it
    // will work as if there are no namespace
    ros::NodeHandle nh_;

    const std::string &GetName() const;
    const std::string &GetType() const;
    Model *GetModel();


    // This function must be overridden for initialization. The YAML Node is
    // passed in for processing by the specific plugin
    virtual void OnInitialize(const YAML::Node &config) = 0;


    // These two functions are called before and after the physics step, the
    // time data contained in timekeeper and ROS simulation time from
    // ros::Time::Now() have been set correctly
    virtual void BeforePhysicsStep(const Timekeeper &timekeeper) {} // time t
    virtual void AfterPhysicsStep(const Timekeeper &timekeeper) {}  // time t + dt


    // helper function to filter contact and returns true if the model is involved
    // in the contact, or false otherwise. If true, entity returns the pointer
    // to the entity that collided with the model, this_fixture returns the
    // fixture in the model, other_fixture returns the fixture in the entity
    bool FilterContact(b2Contact *contact, Entity *&entity,
                       b2Fixture *&this_fixture, b2Fixture *&other_fixture);
    
    // simplified version that just return true / false
    bool FilterContact(b2Contact *contact);


    // Box2D collision callbacks use the FilterContact method to check if the this 
    // plugin's model is involved in the collision, See Box2D documentation for 
    // complete information about these callbacks. These call backs are always
    // called between the BeforePhysicsStep() and AfterPhysicsStep(). The simulation
    // time is always equal to the time at BeforePhysicsStep().

    // Called when two fixtures starts to contact
    virtual void BeginContact(b2Contact *contact) {}  // time t

    // called when two fixtures stops contact
    virtual void EndContact(b2Contact *contact) {}  // time t
    
    // called before solving collision, may be called multiple times in a time step
    virtual void PreSolve(b2Contact *contact, const b2Manifold *oldManifold) {}  // time t

    // called after solving collision, may be called multiple times in a time step
    virtual void PostSolve(b2Contact *contact, const b2ContactImpulse *impulse) {}  // time t
  }

Box2D contact object is generated when two Box2D fixtures collide, it contains
all the information about the collision. Box2D fixtures are the underlying physics 
implementation of footprints in the simulation. From b2Contacts, you can obtain
which two fixtures have collided. From b2Fixtures, we can get the b2Body that the 
fixture belongs to, which also has a one to one relationship to the Flatland's 
Bodies. Each b2Body contain a pointer to a Flatland Body, and each Flatland Body 
has pointer to a entity that is either a layer or a model. Use the Type() method 
to determine if an entity is a model or layer. This is shown as below. 

.. code-block:: Cpp

  // Get the body
  flatland_server::Body *b1 = (flatland_server::Body*) contact->GetFixtureA()->GetBody()->GetUserData();
  flatland_server::Body *b2 = (flatland_server::Body*) contact->GetFixtureB()->GetBody()->GetUserData();

  flatland_server::Entity *e = b1->GetEntity();
  if (e->Type() == flatland_server::Entity::EntityType::LAYER) {
    // entity is Layer
  } else if (e->Type() == flatland_server::Entity::EntityType::MODEL) {
    // entity is Model
  }

Creating the Plugin
-------------------

Say you would want to make a plugin to have a body in the model move at given
constant x, y and yaw rates. This will reside in a package called my_plugins.

1. Create a subclass of ModelPlugin. Note that the name space must be
   flatland_plugins. We must implement the OnInitialize() abstract method, 
   and we need to override the BeforePhysicsStep() to apply the velocity. 
   The velocities are stored in the vel_x, vel_y, and omega members. We also
   need to keep a pointer to the body we are going to apply the velocity to.
   
  .. code-block:: Cpp

    // include/my_plugins/const_velocity_plugin.h

    #include <Box2D/Box2D.h>
    #include <flatland_server/model_plugin.h>
    #include <flatland_server/timekeeper.h>
    #include <flatland_server/body.h>
    #include <yaml-cpp/yaml.h>

    #ifndef FLATLAND_PLUGINS_CONST_VELOCITY
    #define FLATLAND_PLUGINS_CONST_VELOCITY

    namespace flatland_plugins {

    class ConstVelocity : public flatland_server::ModelPlugin {

    public:

      double vel_x, vel_y, omega;
      flatland_server::Body *body;

      void OnInitialize(const YAML::Node &config) override;

      void BeforePhysicsStep(const flatland_server::Timekeeper &timekeeper) override;

    };
    }

    #endif

  We then write the implementation for the ConstVelocity class, the
  PLUGINLIB_EXPORT_CLASS macro is used to register the class within the plugin
  system. YamlReader class is used to help extracting data from the YAML Node.

  .. code-block:: Cpp

    // src/const_velocity_plugin.cpp

    #include <flatland_plugins/laser.h>
    #include <pluginlib/class_list_macros.h>
    #include <flatland_server/yaml_reader.h>
    #include <flatland_server/exceptions.h>

    namespace flatland_plugins {

    void ConstVelocity::OnInitialize(const YAML::Node &config) {
      flatland_server::YamlReader reader(config);

      vel_x = reader.Get<double>("vel_x");
      vel_y = reader.Get<double>("vel_y");
      omega = reader.Get<double>("omega");

      body = GetModel()->GetBody(reader.Get<std::string>("body"));

      // check a valid body is given
      if (body == nullptr) {
        throw flatland_server::YAMLException("Body with with the given name does not exist");
      }
    }

    void ConstVelocity::BeforePhysicsStep(const flatland_server::Timekeeper &timekeeper) {
      body->GetPhysicsBody()->SetLinearVelocity(b2Vec2(vel_x, vel_y));
      body->GetPhysicsBody()->SetAngularVelocity(omega);
    }

    }

    PLUGINLIB_EXPORT_CLASS(flatland_plugins::ConstVelocity,
                          flatland_server::ModelPlugin)

2. Add pluginlib and flatland_server as dependencies in package.xml and 
   CMakeLists.txt. We also need to add the source of the plugin to compile as 
   a library in CMakeLists.txt.

  package.xml:

  .. code-block:: xml

    <depend>flatland_server</depend>
    <depend>pluginlib</depend>

  CMakeLists.txt:

  .. code-block:: cmake

    find_package(catkin REQUIRED COMPONENTS
      pluginlib
      flatland_server
    )

  .. code-block:: cmake

    include_directories(include)
    add_library(my_plugins_lib src/const_velocity_plugin.cpp)

3. At this point, the code should compile, but we can't load the plugin
   dynamically yet because flatland_server would not know its existence. 
   To do this we need to add a flatland_plugins.xml file to list the plugins
   defined in this package and then export it.

   Create a file called flatland_plugins.xml. The <library> tag specifies the 
   compiled library we want to export, note that prefix "lib" is always added 
   to compiled library binaries. The <class> tag declares plugins we want to 
   export. Add as many <class> tags as required for the plugins that needs to 
   be exported. The description of parameter are as follows.

   * **type**: The fully qualified type of the plugin, which is my_plugins::ConstVelocity we just created
   * **base_class**: The fully qualified base class type for the plugin, which will always be flatland_server::ModelPlugin.
   * **description**: A description of what the plugin does

  flatland_plugins.xml:

  .. code-block:: xml

    <library path="lib/libmy_plugins_lib">
      <class type="my_plugins::ConstVelocity" base_class_type="flatland_server::ModelPlugin">
        <description>Constant velocity plugin</description>
      </class>
    </library>
 
  Finally, add the following to package.xml <export> tag to export the plugin.
  The name of the tag should always be flatland_server. And the name of the xml
  file should be the same as the one defined above.

  package.xml:

  .. code-block:: xml

    <export>
      <flatland_server plugin="${prefix}/flatland_plugins.xml" />
    </export>

5. To verify that things are working correctly, build the workspace, source
   devel/setup.bash, and run the following command. You should see the the full
   path to the flatland_plugins.xml file. This means the exporting is configuring
   correctly.

  .. code-block:: bash

    $ rospack plugins --attrib=plugin flatland_server

6. Using a plugin

   To use a model plugin, simply add a plugin entry under plugins as shown in 
   the example model yaml file below. After adding the model to the world, the 
   model should travel at the specified velocities.

  .. code-block:: yaml

    bodies: 
      - name: base
        footprints:
          - type: polygon
            points: [[.5, .5], [-.5, .5], [-.5, -.5], [.5, -.5]]
            density: 1

    plugins:
      - type: ConstVelocity 
        name: const_velocity_drive 
        body: base
        vel_x: 1
        vel_y: 0.2
        omega: -0.5


7. If there are issues, check that PLUGINLIB_EXPORT_CLASS is used to export
   the plugin class, check the spelling of classes, library files, plugin.xml XML 
   tags, and file names to make everything is hooked up correctly.


Model Namespacing
-----------------
Models have a optional namespace parameter. When it is not set, it defaults to
"", and it is equivalent to having no namespace. Namespace allows the simulation
to load multiple of the same model, without worrying about the topic names
and TF frames conflicting between these models. The node handle of model plugins
are initialized with the model's namespace, and the namespace will be automatically
added to all topic names subscribed and advertised. This is shown below.

.. code-block:: Cpp

  nh_ = ros::NodeHandle(model_->namespace_);

To avid conflicts in TF frame IDs, if the plugins choose to publish TF, use
tf::resolve() function to prepend the namespace to **frames on the model** as shown below.

.. code-block:: Cpp

  tf::resolve(GetModel()->NameSpaceTF(frame_id));

NameSpaceTF will not prepend to frame_id's beginning with "/". It will instead strip the leading "/". 

YAML Reader
-----------
Flatland server provides YAML Reader to simplify the process of extracting
data from YAML files. It provides methods to extract scalars, lists, and array
as well as providing error checking, checks for invalid/unused keys, and it 
throws exceptions with messages telling the user what and where the error is. 
Check YamlReader from API documentation, and examples throughout flatland_server
and  flatland_plugins for more details. 

Simulation Time
---------------
Using the launch file provided, ROS will be configured to use simulation time.
One can use ros::Time::now() to get the current time. Simulation time can be
obtained from the timekeeper object, as well as other time related information
such as step size.


Update Timer
------------
It is often desireable to perform updates at a slower rate than what the
simulation is running at. For example, the simulation might be executing in
real time speed at 200Hz, and you wish to publish laser data at 10Hz. This can be
done through the flatland_plugins/UpdateTimer class. The following code
snippet shows how it can be used, and more information and examples can be
obtained from API documentation as well as examples in flatland_plugins.

.. code-block:: Cpp

  #include <flatland_plugins/update_timer.h>

  UpdateTimer update_timer;
  update_timer.SetRate(10);  // set rate in Hz

.. code-block:: Cpp

  void YourPlugin::BeforePhysicsStep(const Timekeeper &timekeeper) {

    // check if an update is REQUIRED
    if (!update_timer.CheckUpdate(timekeeper)) {
      return;
    }

    // the code here will be run at 10Hz
  }