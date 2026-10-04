#include <flatland_controls_plugins/drive_wheel.h>
#include <flatland_controls_plugins/steering_motor.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>

#include <filesystem>
#include <limits>
#include <numbers>

namespace
{
std::unique_ptr<flatland_server::World> MakeWorld(rclcpp::Node::SharedPtr node, bool steering)
{
  const auto root = std::filesystem::path(__FILE__).parent_path();
  const auto path = steering ? root.parent_path().parent_path() /
    "flatland_server/test/load_world_tests/simple_test_A/world.yaml" :
    root / "update_timer_test/world.yaml";
  return std::unique_ptr<flatland_server::World>(flatland_server::World::MakeWorld(node, path.string()));
}

template<typename Plugin>
void SendCommand(
  rclcpp::Node::SharedPtr node, rclcpp::executors::SingleThreadedExecutor & executor,
  const std::shared_ptr<Plugin> & plugin, const std::string & mode, double value)
{
  auto publisher = node->create_publisher<control_msgs::msg::JointCommand>(
    plugin->command_sub_->get_topic_name(), 1);
  control_msgs::msg::JointCommand command;
  command.interface_name = mode;
  command.joint_names = {plugin->GetName()};
  command.values = {value};
  rclcpp::WallRate rate(100);
  for (int attempt = 0; attempt < 50 && !plugin->has_command_; ++attempt) {
    publisher->publish(command);
    executor.spin_some();
    rate.sleep();
  }
  ASSERT_TRUE(plugin->has_command_);
}
}  // namespace

TEST(JointActuatorTest, LoadsFromPluginlib)
{
  pluginlib::ClassLoader<flatland_server::ModelPlugin> loader(
    "flatland_server", "flatland_server::ModelPlugin");
  EXPECT_NE(loader.createSharedInstance("flatland_controls_plugins::DriveWheel"), nullptr);
  EXPECT_NE(loader.createSharedInstance("flatland_controls_plugins::SteeringMotor"), nullptr);
}

TEST(JointActuatorTest, DistributesLoadAndLimitsWheelEffort)
{
  auto node = rclcpp::Node::make_shared("test_drive_wheel");
  auto world = MakeWorld(node, false);
  auto * body = world->models_[0]->GetBody("base")->physics_body_;
  std::vector<std::shared_ptr<flatland_controls_plugins::DriveWheel>> wheels;
  for (double x : {-1.0, 1.0}) {
    YAML::Node config;
    config["body"] = "base";
    config["radius"] = 0.1;
    config["offset"] = std::vector<double>{x, 0.0, 0.0};
    config["mode"] = "effort";
    config["friction"] = 0.5;
    auto wheel = std::make_shared<flatland_controls_plugins::DriveWheel>();
    wheel->Initialize(node, "flatland_controls_plugins::DriveWheel", x < 0 ? "left" : "right", world->models_[0], config);
    world->plugin_manager_.model_plugins_.push_back(wheel);
    wheels.push_back(wheel);
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  SendCommand(node, executor, wheels.front(), "effort", 1000.0);
  EXPECT_FALSE(wheels.back()->has_command_);
  sensor_msgs::msg::JointState::SharedPtr state;
  auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
    wheels.front()->state_pub_->get_topic_name(), rclcpp::SensorDataQoS(),
    [&state](sensor_msgs::msg::JointState::SharedPtr message) {
      if (message->name[0] == "left") {
        state = message;
      }
    });
  flatland_server::Timekeeper timekeeper(node);
  timekeeper.SetMaxStepSize(0.01);
  const double center_x = body->GetWorldCenter().x;
  world->Update(timekeeper);

  EXPECT_NEAR(wheels[0]->load_ + wheels[1]->load_, body->GetMass() * 9.81, 1e-3);
  EXPECT_NEAR(wheels[0]->load_ / (wheels[0]->load_ + wheels[1]->load_),
    std::clamp((1.0 - center_x) / 2.0, 0.0, 1.0), 1e-3);
  EXPECT_GT(body->GetLinearVelocity().x, 0.0);
  EXPECT_LE(wheels[0]->effort_, wheels[0]->friction_ * wheels[0]->load_ * wheels[0]->radius_ + 1e-5);
  rclcpp::WallRate rate(100);
  for (int attempt = 0; attempt < 50 && !state; ++attempt) {
    executor.spin_some();
    rate.sleep();
  }
  ASSERT_NE(state, nullptr);
  EXPECT_EQ(state->name[0], "left");
  EXPECT_GT(state->position[0], 0.0);
  EXPECT_GT(state->velocity[0], 0.0);
  EXPECT_NEAR(state->effort[0], wheels[0]->effort_, 1e-4);
}

TEST(JointActuatorTest, VelocityWheelAcceleratesTowardCommand)
{
  auto node = rclcpp::Node::make_shared("test_velocity_wheel");
  auto world = MakeWorld(node, false);
  YAML::Node config;
  config["body"] = "base";
  config["radius"] = 0.1;
  auto wheel = std::make_shared<flatland_controls_plugins::DriveWheel>();
  wheel->Initialize(node, "flatland_controls_plugins::DriveWheel", "velocity_wheel", world->models_[0], config);
  world->plugin_manager_.model_plugins_.push_back(wheel);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  SendCommand(node, executor, wheel, "velocity", 1.0);
  flatland_server::Timekeeper timekeeper(node);
  timekeeper.SetMaxStepSize(0.01);
  world->Update(timekeeper);
  EXPECT_GT(wheel->position_, 0.0);
  EXPECT_GT(wheel->effort_, 0.0);
}

TEST(JointActuatorTest, LateralGripIsLoadLimited)
{
  auto node = rclcpp::Node::make_shared("test_drive_wheel_grip");
  auto world = MakeWorld(node, false);
  auto * body = world->models_[0]->GetBody("base")->physics_body_;
  body->SetTransform(flatland::b2Vec2(2.0f, 3.0f), 0.0f);
  YAML::Node config;
  config["body"] = "base";
  config["radius"] = 0.1;
  config["friction"] = 0.8;
  config["mode"] = "effort";
  auto wheel = std::make_shared<flatland_controls_plugins::DriveWheel>();
  wheel->Initialize(node, "flatland_controls_plugins::DriveWheel", "grip_test", world->models_[0], config);
  wheel->UpdateGroundContactForces({body->GetMass() * 9.81});
  wheel->command_ = 1000.0;
  wheel->has_command_ = true;
  flatland_server::Timekeeper timekeeper(node);
  timekeeper.SetMaxStepSize(0.01);

  body->SetLinearVelocity(flatland::b2Vec2(0.0f, 0.02f));
  wheel->BeforePhysicsStep(timekeeper);
  EXPECT_GT(wheel->effort_, 0.0);
  world->Update(timekeeper);
  EXPECT_NEAR(body->GetLinearVelocity().y, 0.0, 1e-4);

  body->SetLinearVelocity(flatland::b2Vec2(0.0f, 1.0f));
  wheel->BeforePhysicsStep(timekeeper);
  EXPECT_NEAR(wheel->effort_, 0.0, 1e-4);
  world->Update(timekeeper);
  EXPECT_NEAR(body->GetLinearVelocity().y, 1.0 - 0.8 * 9.81 * 0.01, 1e-4);
}

TEST(JointActuatorTest, DriveWheelRejectsPositionMode)
{
  auto node = rclcpp::Node::make_shared("test_drive_wheel_position_mode");
  auto world = MakeWorld(node, false);
  YAML::Node config;
  config["body"] = "base";
  config["radius"] = 0.1;
  config["mode"] = "position";
  auto wheel = std::make_shared<flatland_controls_plugins::DriveWheel>();
  EXPECT_THROW(
    wheel->Initialize(node, "flatland_controls_plugins::DriveWheel", "unsupported_wheel", world->models_[0], config),
    flatland_server::YAMLException);
}

TEST(JointActuatorTest, FourContactLoadsBalanceCenterOfMass)
{
  auto node = rclcpp::Node::make_shared("test_four_contact_loads");
  auto world = MakeWorld(node, false);
  auto * body = world->models_[0]->GetBody("base")->physics_body_;
  const auto center = body->GetWorldCenter();
  std::vector<std::shared_ptr<flatland_controls_plugins::DriveWheel>> wheels;
  for (double x : {-1.0, 1.0}) {
    for (double y : {-1.0, 1.0}) {
      YAML::Node config;
      config["body"] = "base";
      config["radius"] = 0.1;
      config["offset"] = std::vector<double>{x, y, 0.0};
      auto wheel = std::make_shared<flatland_controls_plugins::DriveWheel>();
      wheel->Initialize(node, "flatland_controls_plugins::DriveWheel", "wheel_" + std::to_string(wheels.size()),
        world->models_[0], config);
      world->plugin_manager_.model_plugins_.push_back(wheel);
      wheels.push_back(wheel);
    }
  }
  flatland_server::Timekeeper timekeeper(node);
  timekeeper.SetMaxStepSize(0.01);
  std::vector<flatland::b2Vec2> points;
  for (const auto & wheel : wheels) {
    points.push_back(wheel->GetContactPoints()[0]);
  }
  world->Update(timekeeper);
  double total = 0.0, moment_x = 0.0, moment_y = 0.0;
  for (size_t index = 0; index < wheels.size(); ++index) {
    const auto & wheel = wheels[index];
    const auto point = points[index];
    total += wheel->load_;
    moment_x += wheel->load_ * point.x;
    moment_y += wheel->load_ * point.y;
    EXPECT_GE(wheel->load_, 0.0);
  }
  EXPECT_NEAR(total, body->GetMass() * 9.81, 1e-3);
  EXPECT_NEAR(moment_x / total, center.x, 1e-3);
  EXPECT_NEAR(moment_y / total, center.y, 1e-3);
}

TEST(JointActuatorTest, SteeringSupportsAllModes)
{
  for (const std::string mode : {"position", "velocity", "effort"}) {
    auto node = rclcpp::Node::make_shared("test_steering_" + mode);
    auto world = MakeWorld(node, true);
    YAML::Node config;
    config["joint"] = "tail_revolute";
    config["mode"] = mode;
    auto motor = std::make_shared<flatland_controls_plugins::SteeringMotor>();
    motor->Initialize(node, "flatland_controls_plugins::SteeringMotor", "steering", world->models_[0], config);
    world->plugin_manager_.model_plugins_.push_back(motor);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    SendCommand(node, executor, motor, mode, mode == "effort" ? 5.0 : 0.5);
    sensor_msgs::msg::JointState::SharedPtr state;
    auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
      motor->state_pub_->get_topic_name(), rclcpp::SensorDataQoS(),
      [&state](sensor_msgs::msg::JointState::SharedPtr message) { state = message; });
    flatland_server::Timekeeper timekeeper(node);
    timekeeper.SetMaxStepSize(0.01);
    for (int step = 0; step < 10; ++step) {
      world->Update(timekeeper);
    }
    EXPECT_GT(motor->joint_->GetJointAngle(), 0.0) << mode;
    rclcpp::WallRate rate(100);
    for (int attempt = 0; attempt < 50 && !state; ++attempt) {
      executor.spin_some();
      rate.sleep();
    }
    ASSERT_NE(state, nullptr);
    EXPECT_EQ(state->name[0], motor->GetName());
    EXPECT_GT(state->position[0], 0.0);
    EXPECT_GT(state->velocity[0], 0.0);
  }
}

TEST(JointActuatorTest, SteeringRequiresPluginNameInCommand)
{
  auto node = rclcpp::Node::make_shared("test_steering_command_target");
  auto world = MakeWorld(node, true);
  YAML::Node config;
  config["joint"] = "tail_revolute";
  auto motor = std::make_shared<flatland_controls_plugins::SteeringMotor>();
  motor->Initialize(node, "flatland_controls_plugins::SteeringMotor", "steering_plugin", world->models_[0], config);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto publisher = node->create_publisher<control_msgs::msg::JointCommand>(
    motor->command_sub_->get_topic_name(), 1);
  control_msgs::msg::JointCommand command;
  command.interface_name = "position";
  command.joint_names = {"tail_revolute", "other_plugin"};
  command.values = {0.5, 0.7};
  rclcpp::WallRate rate(100);
  for (int attempt = 0; attempt < 20; ++attempt) {
    publisher->publish(command);
    executor.spin_some();
    rate.sleep();
  }
  EXPECT_FALSE(motor->has_command_);

  command.joint_names = {"other_plugin", motor->GetName()};
  command.values = {0.7, 0.5};
  for (int attempt = 0; attempt < 20 && !motor->has_command_; ++attempt) {
    publisher->publish(command);
    executor.spin_some();
    rate.sleep();
  }
  EXPECT_TRUE(motor->has_command_);
  EXPECT_DOUBLE_EQ(motor->command_, 0.5);
}

TEST(JointActuatorTest, SteeringLimitsRotation)
{
  auto node = rclcpp::Node::make_shared("test_steering_limits");
  auto world = MakeWorld(node, true);
  auto * tail = world->models_[0]->GetBody("tail")->physics_body_;
  tail->SetTransform(tail->GetPosition(), 0.0f);
  YAML::Node config;
  config["joint"] = "tail_revolute";
  config["limit"]["lower"] = -0.1;
  config["limit"]["upper"] = 0.2;
  auto motor = std::make_shared<flatland_controls_plugins::SteeringMotor>();
  motor->Initialize(node, "flatland_controls_plugins::SteeringMotor", "limited_steering", world->models_[0], config);
  world->plugin_manager_.model_plugins_.push_back(motor);
  ASSERT_TRUE(motor->joint_->IsLimitEnabled());
  EXPECT_NEAR(motor->joint_->GetLowerLimit(), -0.1, 1e-6);
  EXPECT_NEAR(motor->joint_->GetUpperLimit(), 0.2, 1e-6);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  SendCommand(node, executor, motor, "position", 1.0);
  flatland_server::Timekeeper timekeeper(node);
  timekeeper.SetMaxStepSize(0.01);
  for (int step = 0; step < 30; ++step) {
    world->Update(timekeeper);
    EXPECT_GE(motor->joint_->GetJointAngle(), -0.12) << step;
    EXPECT_LE(motor->joint_->GetJointAngle(), 0.22) << step;
  }
  EXPECT_GT(motor->joint_->GetJointAngle(), 0.0);
}

TEST(JointActuatorTest, SteeringRejectsInvalidLimits)
{
  auto node = rclcpp::Node::make_shared("test_invalid_steering_limits");
  auto world = MakeWorld(node, true);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const auto & [lower, upper] : std::vector<std::pair<double, double>>{
      {nan, 0.2}, {-0.2, nan}, {0.2, 0.2}, {0.3, 0.2},
      {-std::numbers::pi, 0.2}, {-0.2, std::numbers::pi},
      {-0.2, std::numeric_limits<double>::infinity()}}) {
    YAML::Node config;
    config["joint"] = "tail_revolute";
    config["limit"]["lower"] = lower;
    config["limit"]["upper"] = upper;
    auto motor = std::make_shared<flatland_controls_plugins::SteeringMotor>();
    EXPECT_THROW(
      motor->Initialize(node, "flatland_controls_plugins::SteeringMotor", "invalid_steering", world->models_[0], config),
      flatland_server::YAMLException);
  }
}

TEST(JointActuatorTest, SteeringAcceptsNoLimits)
{
  auto node = rclcpp::Node::make_shared("test_unlimited_steering");
  auto world = MakeWorld(node, true);
  for (bool explicit_nan : {false, true}) {
    YAML::Node config;
    config["joint"] = "tail_revolute";
    if (explicit_nan) {
      config["limit"]["lower"] = std::numeric_limits<double>::quiet_NaN();
      config["limit"]["upper"] = std::numeric_limits<double>::quiet_NaN();
    }
    auto motor = std::make_shared<flatland_controls_plugins::SteeringMotor>();
    motor->Initialize(node, "flatland_controls_plugins::SteeringMotor", "unlimited_steering", world->models_[0], config);
    EXPECT_FALSE(motor->joint_->IsLimitEnabled());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}