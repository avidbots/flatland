#include <flatland_plugins/drive_wheel.h>
#include <flatland_plugins/steering_motor.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <memory>
#include <string>

TEST(ExampleWorlds, LoadAndStep)
{
  const auto worlds = std::filesystem::path(__FILE__).parent_path().parent_path() / "worlds";
  for (const auto & [name, plugin_count] :
    std::array<std::pair<std::string, size_t>, 6>{{
      {"caster_diff", 3}, {"a300_diff", 4}, {"2910_swerve", 8},
      {"rear_drive_ackermann", 6}, {"front_drive_tricycle", 4},
      {"articulated_204g", 5}}}) {
    SCOPED_TRACE(name);
    auto node = rclcpp::Node::make_shared("example_" + name);
    auto world = std::unique_ptr<flatland_server::World>(
      flatland_server::World::MakeWorld(node, (worlds / (name + ".world.yaml")).string()));
    ASSERT_EQ(world->models_.size(), 1u);
    EXPECT_EQ(world->plugin_manager_.model_plugins_.size(), plugin_count);
    flatland_server::Timekeeper timekeeper(node);
    timekeeper.SetMaxStepSize(0.01);
    world->Update(timekeeper);
  }
}

TEST(ExampleWorlds, WheelBearingBodiesHavePlausibleMass)
{
  const auto worlds = std::filesystem::path(__FILE__).parent_path().parent_path() / "worlds";
  auto node = rclcpp::Node::make_shared("example_mass_distribution");
  auto caster = std::unique_ptr<flatland_server::World>(flatland_server::World::MakeWorld(
    node, (worlds / "caster_diff.world.yaml").string()));
  auto * caster_model = caster->models_[0];
  const double caster_mass = caster_model->GetBody("chassis")->physics_body_->GetMass();
  const double platform_mass = caster_model->GetBody("caster_platform")->physics_body_->GetMass();
  EXPECT_NEAR(caster_mass + platform_mass, 4.0, 0.5);
  EXPECT_GT(platform_mass, 0.15);
  EXPECT_LT(platform_mass, 0.5);

  auto car = std::unique_ptr<flatland_server::World>(flatland_server::World::MakeWorld(
    node, (worlds / "rear_drive_ackermann.world.yaml").string()));
  auto * car_model = car->models_[0];
  const double chassis_mass = car_model->GetBody("chassis")->physics_body_->GetMass();
  EXPECT_GT(chassis_mass, 150.0);
  EXPECT_LT(chassis_mass, 300.0);
  double mass = 0.0;
  double weighted_x = 0.0;
  for (const auto * body : car_model->GetBodies()) {
    const auto * physics = body->physics_body_;
    mass += physics->GetMass();
    weighted_x += physics->GetMass() * physics->GetWorldCenter().x;
  }
  EXPECT_LT(weighted_x / mass, 0.0);
  EXPECT_GT(weighted_x / mass, -0.15);
  for (const auto * name : {"front_left_knuckle", "front_right_knuckle"}) {
    const double wheel_mass = car_model->GetBody(name)->physics_body_->GetMass();
    EXPECT_GT(wheel_mass, 5.0);
    EXPECT_LT(wheel_mass, 12.0);
  }
}

TEST(ExampleWorlds, AckermannAndCasterSteerUnderPower)
{
  const auto worlds = std::filesystem::path(__FILE__).parent_path().parent_path() / "worlds";
  for (const auto & name : {"rear_drive_ackermann", "caster_diff"}) {
    SCOPED_TRACE(name);
    auto node = rclcpp::Node::make_shared(std::string("drive_") + name);
    auto world = std::unique_ptr<flatland_server::World>(flatland_server::World::MakeWorld(
      node, (worlds / (std::string(name) + ".world.yaml")).string()));
    for (const auto & plugin : world->plugin_manager_.model_plugins_) {
      if (auto wheel = std::dynamic_pointer_cast<flatland_plugins::DriveWheel>(plugin)) {
        wheel->command_ = name == std::string("caster_diff") && wheel->GetName() == "right_wheel" ?
          5.0 : 3.0;
        wheel->has_command_ = true;
      }
      if (auto steer = std::dynamic_pointer_cast<flatland_plugins::SteeringMotor>(plugin)) {
        steer->command_ = 0.25;
        steer->has_command_ = true;
      }
    }
    auto * body = world->models_[0]->GetBody("chassis")->physics_body_;
    const auto start = body->GetWorldCenter();
    flatland_server::Timekeeper timekeeper(node);
    timekeeper.SetMaxStepSize(0.01);
    for (int step = 0; step < 200; ++step) {
      world->Update(timekeeper);
    }
    const auto end = body->GetWorldCenter();
    EXPECT_GT(end.x - start.x, 0.2);
    EXPECT_GT(body->GetAngle(), 0.05);
    EXPECT_GT(end.y - start.y, 0.01);
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