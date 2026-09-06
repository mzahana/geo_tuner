#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "geo_tuner/quad_sim.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<geo_tuner::QuadSim>());
  rclcpp::shutdown();
  return 0;
}
