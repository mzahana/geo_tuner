#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "geo_tuner/tuning_conductor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<geo_tuner::TuningConductor>());
  rclcpp::shutdown();
  return 0;
}
