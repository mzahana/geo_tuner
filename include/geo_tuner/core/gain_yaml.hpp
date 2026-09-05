// YAML emission for a designed gain set: the two files the field
// procedure copies onto the vehicle.
#ifndef GEO_TUNER__CORE__GAIN_YAML_HPP_
#define GEO_TUNER__CORE__GAIN_YAML_HPP_

#include <yaml-cpp/yaml.h>

#include "geo_tuner/core/gain_design.hpp"

namespace geo_tuner
{

/// Full ros__parameters tree for geometric_controller.yaml.
YAML::Node geometric_controller_yaml(
  const GainSet & g, double mass, double max_tilt_angle = 0.52, double max_accel = 5.0);

YAML::Node geometric_mavros_yaml(const GainSet & g);

}  // namespace geo_tuner

#endif  // GEO_TUNER__CORE__GAIN_YAML_HPP_
