#include <flatland_plugins/gps.h>
#include <flatland_server/model_plugin.h>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

TEST(GpsPluginTest, load_test) {
  pluginlib::ClassLoader<flatland_server::ModelPlugin> loader(
      "flatland_server", "flatland_server::ModelPlugin");

  try {
    boost::shared_ptr<flatland_server::ModelPlugin> plugin =
        loader.createInstance("flatland_plugins::Gps");
  } catch (pluginlib::PluginlibException& e) {
    FAIL() << "Failed to load GPS plugin. " << e.what();
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  ros::init(argc, argv, "gps_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
