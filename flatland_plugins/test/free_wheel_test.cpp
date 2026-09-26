#include <flatland_plugins/free_wheel.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>

#include <filesystem>
#include <numbers>

TEST(FreeWheelTest, LoadsFromPluginlib)
{
  pluginlib::ClassLoader<flatland_server::ModelPlugin> loader(
    "flatland_server", "flatland_server::ModelPlugin");
  EXPECT_NE(loader.createSharedInstance("flatland_plugins::FreeWheel"), nullptr);
}

TEST(FreeWheelTest, ResistsLateralMotionAndPublishesEncoder)
{
  auto node = rclcpp::Node::make_shared("test_free_wheel");
  auto world_path = std::filesystem::path(__FILE__).parent_path() / "update_timer_test/world.yaml";
  std::unique_ptr<flatland_server::World> world(
    flatland_server::World::MakeWorld(node, world_path.string()));
  auto * body = world->models_[0]->GetBody("base")->physics_body_;
  body->SetTransform(flatland::b2Vec2(2.0f, 3.0f), 0.0f);

  YAML::Node config;
  config["body"] = "base";
  config["offset"] = std::vector<double>{1.0, 0.0, std::numbers::pi / 2};
  config["radius"] = 0.1;
  config["lateral_resistance"] = 10.0;
  config["encoder_topic"] = "wheel/encoder";
  auto plugin = std::make_shared<flatland_plugins::FreeWheel>();
  plugin->Initialize(node, "FreeWheel", "test_wheel", world->models_[0], config);
  world->plugin_manager_.model_plugins_.push_back(plugin);

  std_msgs::msg::Float64::SharedPtr encoder;
  auto subscription = node->create_subscription<std_msgs::msg::Float64>(
    plugin->encoder_pub_->get_topic_name(), 1,
    [&encoder](std_msgs::msg::Float64::SharedPtr message) { encoder = message; });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  body->SetLinearVelocity(flatland::b2Vec2(1.0f, 0.0f));
  body->SetAngularVelocity(1.0f);
  const double expected_encoder =
    body->GetLinearVelocityFromLocalPoint(flatland::b2Vec2(1.0f, 0.0f)).y / 0.1;
  flatland_server::Timekeeper timekeeper(node);
  timekeeper.SetMaxStepSize(0.01);
  world->Update(timekeeper);
  EXPECT_LT(body->GetLinearVelocity().x, 1.0f);

  rclcpp::WallRate rate(100);
  for (int attempt = 0; attempt < 30 && !encoder; ++attempt) {
    executor.spin_some();
    rate.sleep();
  }
  ASSERT_NE(encoder, nullptr);
  EXPECT_NEAR(encoder->data, expected_encoder, 1e-4);
}

TEST(FreeWheelTest, RejectsNonpositiveRadius)
{
  auto node = rclcpp::Node::make_shared("test_free_wheel_radius");
  auto world_path = std::filesystem::path(__FILE__).parent_path() / "update_timer_test/world.yaml";
  std::unique_ptr<flatland_server::World> world(
    flatland_server::World::MakeWorld(node, world_path.string()));
  YAML::Node config;
  config["body"] = "base";
  config["radius"] = 0.0;
  auto plugin = std::make_shared<flatland_plugins::FreeWheel>();
  EXPECT_THROW(
    plugin->Initialize(node, "FreeWheel", "test_wheel", world->models_[0], config),
    flatland_server::YAMLException);
}

TEST(FreeWheelTest, EncoderIsOptional)
{
  auto node = rclcpp::Node::make_shared("test_free_wheel_no_encoder");
  auto world_path = std::filesystem::path(__FILE__).parent_path() / "update_timer_test/world.yaml";
  std::unique_ptr<flatland_server::World> world(
    flatland_server::World::MakeWorld(node, world_path.string()));
  YAML::Node config;
  config["body"] = "base";
  config["radius"] = 0.1;
  auto plugin = std::make_shared<flatland_plugins::FreeWheel>();
  plugin->Initialize(node, "FreeWheel", "test_wheel", world->models_[0], config);
  EXPECT_EQ(plugin->encoder_pub_, nullptr);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}