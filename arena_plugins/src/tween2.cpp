#include <arena_plugins/tween2.h>
#include <pluginlib/class_list_macros.h>

#include <cmath>
namespace flatland_plugins {
std::map<std::string, Tween2::ModeType_> Tween2::mode_strings_ = {
    {"yoyo", Tween2::ModeType_::YOYO},
    {"loop", Tween2::ModeType_::LOOP},
    {"once", Tween2::ModeType_::ONCE},
};

std::map<std::string, Tween2::EasingType_> Tween2::easing_strings_ = {
    {"linear", Tween2::EasingType_::linear},
    {"quadraticIn", Tween2::EasingType_::quadraticIn},
    {"quadraticOut", Tween2::EasingType_::quadraticOut},
    {"quadraticInOut", Tween2::EasingType_::quadraticInOut},
    {"cubicIn", Tween2::EasingType_::cubicIn},
    {"cubicOut", Tween2::EasingType_::cubicOut},
    {"cubicInOut", Tween2::EasingType_::cubicInOut},
    {"quarticIn", Tween2::EasingType_::quarticIn},
    {"quarticOut", Tween2::EasingType_::quarticOut},
    {"quarticInOut", Tween2::EasingType_::quarticInOut},
    {"quinticIn", Tween2::EasingType_::quinticIn},
    {"quinticOut", Tween2::EasingType_::quinticOut},
    {"quinticInOut", Tween2::EasingType_::quinticInOut},
    // { "sinuisodal", Tween2::EasingType_::sinuisodal },
    {"exponentialIn", Tween2::EasingType_::exponentialIn},
    {"exponentialOut", Tween2::EasingType_::exponentialOut},
    {"exponentialInOut", Tween2::EasingType_::exponentialInOut},
    {"circularIn", Tween2::EasingType_::circularIn},
    {"circularOut", Tween2::EasingType_::circularOut},
    {"circularInOut", Tween2::EasingType_::circularInOut},
    {"backIn", Tween2::EasingType_::backIn},
    {"backOut", Tween2::EasingType_::backOut},
    {"backInOut", Tween2::EasingType_::backInOut},
    {"elasticIn", Tween2::EasingType_::elasticIn},
    {"elasticOut", Tween2::EasingType_::elasticOut},
    {"elasticInOut", Tween2::EasingType_::elasticInOut},
    {"bounceIn", Tween2::EasingType_::bounceIn},
    {"bounceOut", Tween2::EasingType_::bounceOut},
    {"bounceInOut", Tween2::EasingType_::bounceInOut}};

void Tween2::OnInitialize(const YAML::Node& config) {
  triggered_ = false;
  YamlReader reader(config);
  std::string body_name = reader.Get<std::string>("body");
  // the topic name the move the object to the start point.
  std::string move_to_start_pos_topic =
      reader.Get<std::string>("move_to_start_pos_topic", "");
  if (move_to_start_pos_topic != "") {
    reset_sub_ = nh_.subscribe(move_to_start_pos_topic, 1,
                               &Tween2::MoveToStartPosCallback, this);
    ROS_INFO_STREAM("Tween2 subscribe to " << move_to_start_pos_topic);
  }

  watcher_zones_ = reader.GetList<Pose>("trigger_zones", 0, 5);
  std::string robot_odom_topic =
      reader.Get<std::string>("robot_odom_topic", "");
  if (watcher_zones_.size() > 0) {
    if (robot_odom_topic == "") {
      ROS_ERROR("No robot odom topic is defined, object will keep be static");
    } else {
      trigger_sub_ = nh_.subscribe(robot_odom_topic, 1,
                                   &Tween2::ChangeTriggerStatusCallback, this);
    }
  } else {
    triggered_ = true;
  }

  // reciprocal, loop, or oneshot
  std::string mode = reader.Get<std::string>("mode", "yoyo");
  float linear_velocity = reader.Get<float>("linear_velocity", 1.0f);
  // minimum 2 points, maximum 30 points.
  std::vector<Pose> waypoints = reader.GetList<Pose>("waypoints", 1, 30);
  auto is_waypoint_relative_ = reader.Get<bool>("is_waypoint_relative", true);

  body_ = GetModel()->GetBody(body_name);
  if (body_ == nullptr) {
    throw YAMLException("Body with name " + Q(body_name) + " does not exist");
  }
  start_ = Pose(body_->physics_body_->GetPosition().x,
                body_->physics_body_->GetPosition().y,
                body_->physics_body_->GetAngle());
  // Validate the mode selection
  if (!mode_strings_.count(mode)) {
    throw YAMLException("Mode " + mode + " does not exist");
  }
  Tween2::EasingType_ easing_type;
  std::string easing = reader.Get<std::string>("easing", "linear");
  if (!Tween2::easing_strings_.count(easing)) {
    throw YAMLException("Mode " + mode + " does not exist");
  }
  easing_type = Tween2::easing_strings_.at(easing);

  mode_ = Tween2::mode_strings_.at(mode);
  Pose last_pos;
  if (is_waypoint_relative_) {
    tween_ = tweeny::from(0.0, 0.0, 0.0);
    last_pos = Pose(0.0, 0.0, 0.0);
  } else {
    tween_ = tweeny::from(start_.x, start_.y, start_.theta);
    last_pos = start_;
  }
  for (auto& pos : waypoints) {
    float dist = sqrt(pow(pos.x - last_pos.x, 2) + pow(pos.y - last_pos.y, 2));
    float duration = dist / linear_velocity;
    tween_.to(pos.x, pos.y, pos.theta);
    tween_.during(static_cast<uint32>(duration * 1000));
    // copied from flatland Tween2, tweeny dont implement a interface,
    // therefore those struct can not be stored in a container.
    // This is clumsy but because tweeny used structs for each tweening rather
    // than subclasses
    // I believe that this is the best way to do this
    switch (easing_type) {
      case Tween2::EasingType_::linear:
        tween_ = tween_.via(tweeny::easing::linear);
        break;
      case Tween2::EasingType_::quadraticIn:
        tween_ = tween_.via(tweeny::easing::quadraticIn);
        break;
      case Tween2::EasingType_::quadraticOut:
        tween_ = tween_.via(tweeny::easing::quadraticOut);
        break;
      case Tween2::EasingType_::quadraticInOut:
        tween_ = tween_.via(tweeny::easing::quadraticInOut);
        break;
      case Tween2::EasingType_::cubicIn:
        tween_ = tween_.via(tweeny::easing::cubicIn);
        break;
      case Tween2::EasingType_::cubicOut:
        tween_ = tween_.via(tweeny::easing::cubicOut);
        break;
      case Tween2::EasingType_::cubicInOut:
        tween_ = tween_.via(tweeny::easing::cubicInOut);
        break;
      case Tween2::EasingType_::quarticIn:
        tween_ = tween_.via(tweeny::easing::quarticIn);
        break;
      case Tween2::EasingType_::quarticOut:
        tween_ = tween_.via(tweeny::easing::quarticOut);
        break;
      case Tween2::EasingType_::quarticInOut:
        tween_ = tween_.via(tweeny::easing::quarticInOut);
        break;
      case Tween2::EasingType_::quinticIn:
        tween_ = tween_.via(tweeny::easing::quinticIn);
        break;
      case Tween2::EasingType_::quinticOut:
        tween_ = tween_.via(tweeny::easing::quinticOut);
        break;
      case Tween2::EasingType_::quinticInOut:
        tween_ = tween_.via(tweeny::easing::quinticInOut);
        break;
      // case Tween2::EasingType_::sinuisodal:
      //   tween_ = tween_.via(tweeny::easing::sinuisodal);
      //   break;
      case Tween2::EasingType_::exponentialIn:
        tween_ = tween_.via(tweeny::easing::exponentialIn);
        break;
      case Tween2::EasingType_::exponentialOut:
        tween_ = tween_.via(tweeny::easing::exponentialOut);
        break;
      case Tween2::EasingType_::exponentialInOut:
        tween_ = tween_.via(tweeny::easing::exponentialInOut);
        break;
      case Tween2::EasingType_::circularIn:
        tween_ = tween_.via(tweeny::easing::circularIn);
        break;
      case Tween2::EasingType_::circularOut:
        tween_ = tween_.via(tweeny::easing::circularOut);
        break;
      case Tween2::EasingType_::circularInOut:
        tween_ = tween_.via(tweeny::easing::circularInOut);
        break;
      case Tween2::EasingType_::backIn:
        tween_ = tween_.via(tweeny::easing::backIn);
        break;
      case Tween2::EasingType_::backOut:
        tween_ = tween_.via(tweeny::easing::backOut);
        break;
      case Tween2::EasingType_::backInOut:
        tween_ = tween_.via(tweeny::easing::backInOut);
        break;
      case Tween2::EasingType_::elasticIn:
        tween_ = tween_.via(tweeny::easing::elasticIn);
        break;
      case Tween2::EasingType_::elasticOut:
        tween_ = tween_.via(tweeny::easing::elasticOut);
        break;
      case Tween2::EasingType_::elasticInOut:
        tween_ = tween_.via(tweeny::easing::elasticInOut);
        break;
      case Tween2::EasingType_::bounceIn:
        tween_ = tween_.via(tweeny::easing::bounceIn);
        break;
      case Tween2::EasingType_::bounceOut:
        tween_ = tween_.via(tweeny::easing::bounceOut);
        break;
      case Tween2::EasingType_::bounceInOut:
        tween_ = tween_.via(tweeny::easing::bounceInOut);
        break;
      default:
        throw new Exception("Unknown easing type!");
    }
  }
  // Make sure there are no unused keys
  reader.EnsureAccessedAllKeys();

  ROS_DEBUG_NAMED("Tween2",
                  "Initialized with params body(%p %s) "
                  "start ({%f,%f,%f}) "
                  "end ({%f,%f,%f}) "
                  "duration %f "
                  "mode: %s [%d] "
                  "easing: %s\n",
                  body_, body_->name_.c_str(), start_.x, start_.y, start_.theta,
                  delta_.x, delta_.y, delta_.theta, duration_, mode.c_str(),
                  (int)mode_, easing.c_str());
}

void Tween2::BeforePhysicsStep(const Timekeeper& timekeeper) {
  if (triggered_) {
    std::array<double, 3> v =
        tween_.step((uint32)(timekeeper.GetStepSize() * 1000.0));
    ROS_DEBUG_THROTTLE_NAMED(1.0, "Tween2",
                             "value %f,%f,%f step %f progress %f", v[0], v[1],
                             v[2], timekeeper.GetStepSize(), tween_.progress());
    body_->physics_body_->SetTransform(b2Vec2(start_.x + v[0], start_.y + v[1]),
                                       start_.theta + v[2]);
    // Tell Box2D to update the AABB and check for collisions for this object
    body_->physics_body_->SetAwake(true);

    // Yoyo back and forth
    if (mode_ == Tween2::ModeType_::YOYO) {
      if (tween_.progress() >= 1.0f) {
        tween_.backward();
      } else if (tween_.progress() <= 0.001f) {
        tween_.forward();
      }
    }

    // Teleport back in loop mode
    if (mode_ == Tween2::ModeType_::LOOP) {
      if (tween_.progress() >= 1.0f) {
        tween_.seek(0);
      }
    }
  }
}
void Tween2::ChangeTriggerStatusCallback(const nav_msgs::Odometry& msg) {
  if (!triggered_) {
    auto& robot_pos = msg.pose.pose.position;
    for (auto& watcher_zone : watcher_zones_) {
      // distance robot to watcher center
      float dis_robot_watcher = sqrt(pow(robot_pos.x - watcher_zone.x, 2) +
                                     pow(robot_pos.y - watcher_zone.y, 2));
      // ROS_INFO_STREAM("curr_dist:\t" << dis_robot_watcher << "threshold:\t"
      //                                << watcher_zone.theta);
      // TODO use Vec3 can not compile, need to figure out whats the problem
      if (dis_robot_watcher <= watcher_zone.theta) {
        triggered_ = true;
        break;
      }
    }
  }
}

}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::Tween2, flatland_server::ModelPlugin)