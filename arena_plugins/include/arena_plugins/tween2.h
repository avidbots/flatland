#ifndef ARENA_PLUGINS_TWEEN2
#define ARENA_PLUGINS_TWEEN2
#include <flatland_plugins/tween.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
using namespace flatland_server;
namespace flatland_plugins {
// If Tween2 inherited from Tween1 then it can not be loaded which is very
// weired.
class Tween2 : public ModelPlugin {
 private:
  bool triggered_;
  ros::Subscriber trigger_sub_;
  ros::Subscriber reset_sub_;
  // TODO better use other type.
  std::vector<Pose> watcher_zones_;

 public:
  Body* body_;      // The body this plugin is attached to
  Pose start_;      // The start pose of the model
  Pose delta_;      // The maximum change
  float duration_;  // Seconds to enact change over
  tweeny::tween<double, double, double> tween_;  // The tween object (x,y,theta)

  // The three different operating modes
  enum class ModeType_ {
    YOYO,  // tween up to delta_, then down again, and repeat
    LOOP,  // tween up to delta_, then teleport back to start_
    ONCE,  // tween up to delta_ then stay there
  };
  ModeType_ mode_;
  static std::map<std::string, ModeType_> mode_strings_;

  enum class EasingType_ {
    linear,
    quadraticIn,
    quadraticOut,
    quadraticInOut,
    cubicIn,
    cubicOut,
    cubicInOut,
    quarticIn,
    quarticOut,
    quarticInOut,
    quinticIn,
    quinticOut,
    quinticInOut,
    // sinuisodal,
    exponentialIn,
    exponentialOut,
    exponentialInOut,
    circularIn,
    circularOut,
    circularInOut,
    backIn,
    backOut,
    backInOut,
    elasticIn,
    elasticOut,
    elasticInOut,
    bounceIn,
    bounceOut,
    bounceInOut
  };
  static std::map<std::string, EasingType_> easing_strings_;
  /**
   * @name          OnInitialize
   * @brief         Initialize the plugin with yaml
   * compared with flatland default tween plugin, this plugin
   * can move the object along a set of waypoints with the constant_velocity
   * @param[in]     config The plugin YAML node
   */
  void OnInitialize(const YAML::Node& config) override;

  // /**
  //  * @name          BeforePhysicsStep
  //  * @brief         override the BeforePhysicsStep method
  //  * @param[in]     config The plugin YAML node
  //  */
  void BeforePhysicsStep(const Timekeeper& timekeeper) override;
  /**
   * @brief  move the object to the start position.
   */
  inline void MoveToStartPosCallback(const std_msgs::Empty& msg) {
    tween_.seek(0);
    body_->physics_body_->SetTransform(b2Vec2(start_.x, start_.y),
                                       start_.theta);
    // if no watcher added robot can move right after reset.
    if (watcher_zones_.size() == 0) {
      triggered_ = true;
    } else {
      triggered_ = false;
    }
  };
  void ChangeTriggerStatusCallback(const nav_msgs::Odometry& msg);
};

};  // namespace flatland_plugins

#endif