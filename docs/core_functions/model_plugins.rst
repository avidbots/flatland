Writing Model Plugins
=====================
Flatland is designed with extensibility in mind. Users can write their own Plugins
to tailor the simulator for their specific purposes. A Model Plugin is a plugin
that is attached to a specific model and perform specific functions for
that model. 

The plugin system is implemented using `ROS pluginlib <http://wiki.ros.org/pluginlib>`_.
All model plugins must implement the base class ModelPlugin with the critical 
class memebers listed below.

.. code-block:: Cpp

  class ModelPlugin {
  public:
    std::string type_;  // type of the plugin
    std::string name_;  // 
    ros::NodeHandle nh_;  
    Model *model_;

    bool FilterContact(b2Contact *contact, Entity *&entity,
                      b2Fixture *&this_fixture, b2Fixture *&other_fixture);

    bool FilterContact(b2Contact *contact);

    virtual void OnInitialize(const YAML::Node &config) = 0;

    virtual void BeforePhysicsStep(const Timekeeper &timekeeper) {}

    virtual void AfterPhysicsStep(const Timekeeper &timekeeper) {}

    virtual void BeginContact(b2Contact *contact) {}

    virtual void EndContact(b2Contact *contact) {}

    virtual void PreSolve(b2Contact *contact, const b2Manifold *oldManifold) {}

    virtual void PostSolve(b2Contact *contact, const b2ContactImpulse *impulse) {}
  }