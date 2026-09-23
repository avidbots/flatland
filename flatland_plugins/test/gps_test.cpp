#include <flatland_server/model_plugin.h>
#include <gtest/gtest.h>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

TEST(GpsPluginTest, load_test)
{
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_gps_plugin");
  pluginlib::ClassLoader<flatland_server::ModelPlugin> loader(
    "flatland_server", "flatland_server::ModelPlugin");

  try {
    std::shared_ptr<flatland_server::ModelPlugin> plugin =
      loader.createSharedInstance("flatland_plugins::Gps");
  } catch (pluginlib::PluginlibException & e) {
    FAIL() << "Failed to load GPS plugin. " << e.what();
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
