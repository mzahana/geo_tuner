#include "geo_tuner/core/gain_yaml.hpp"

#include "geo_tuner/core/yaml_double.hpp"

namespace geo_tuner
{

YAML::Node geometric_controller_yaml(
  const GainSet & g, double mass, double max_tilt_angle, double max_accel)
{
  YAML::Node pos;
  pos["x"] = yaml_double(g.kx[0]);
  pos["y"] = yaml_double(g.kx[1]);
  pos["z"] = yaml_double(g.kx[2]);
  YAML::Node vel;
  vel["x"] = yaml_double(g.kv[0]);
  vel["y"] = yaml_double(g.kv[1]);
  vel["z"] = yaml_double(g.kv[2]);
  YAML::Node zero;
  zero["x"] = yaml_double(0.0);
  zero["y"] = yaml_double(0.0);
  zero["z"] = yaml_double(0.0);

  YAML::Node gains;
  gains["pos"] = pos;
  gains["vel"] = vel;
  // Integral off during tuning; the implementation accumulates
  // per-callback (no dt) so keep tiny if used.
  gains["ki"] = YAML::Clone(zero);
  gains["kib"] = YAML::Clone(zero);

  YAML::Node drag;
  drag["kd"] = YAML::Clone(zero);

  YAML::Node params;
  params["mass"] = yaml_double(mass);
  params["use_external_yaw"] = true;
  params["gains"] = gains;
  params["drag"] = drag;
  params["attctrl_tau"] = yaml_double(g.attctrl_tau);
  params["max_pos_int"] = yaml_double(0.5);
  params["mas_pos_int_b"] = yaml_double(0.5);  // (sic) param name in the node
  params["max_tilt_angle"] = yaml_double(max_tilt_angle);
  params["max_accel"] = yaml_double(max_accel);
  params["yaw_gain"] = yaml_double(0.4);

  YAML::Node node;
  node["geometric_controller_node"]["ros__parameters"] = params;
  return node;
}

YAML::Node geometric_mavros_yaml(const GainSet & g)
{
  YAML::Node params;
  params["num_props"] = 4;
  params["kf"] = yaml_double(1.0);
  params["max_thrust"] = yaml_double(g.max_thrust);
  params["lin_cof_a"] = yaml_double(1.0);
  params["lin_int_b"] = yaml_double(0.0);
  params["se3_cmd_timeout"] = yaml_double(0.25);

  YAML::Node node;
  node["geometric_mavros_node"]["ros__parameters"] = params;
  return node;
}

}  // namespace geo_tuner
