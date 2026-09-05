// CLI: compute geometric controller gains from vehicle characteristics.
//
// Usage:
//     geo-tuner-design --mass 2.5 --hover-throttle 0.45
//         [--attctrl-tau 0.3] [--zeta 0.95] [--latency 0.08] [--wn 1.6]
//         [--out-dir config_out]
//
// Writes geometric_controller.yaml and geometric_mavros.yaml, prints the
// design summary with every constraint that shaped the result.
#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <string>

#include "geo_tuner/core/gain_design.hpp"
#include "geo_tuner/core/gain_yaml.hpp"

namespace
{

void usage(const char * prog)
{
  std::cout <<
    "usage: " << prog << " --mass MASS --hover-throttle HT\n"
    "                       [--attctrl-tau 0.3] [--zeta 0.95] [--latency 0.08]\n"
    "                       [--separation 4.0] [--wn WN] [--max-tilt 0.52]\n"
    "                       [--max-accel 5.0] [--out-dir .]\n\n"
    "Design geometric controller gains from vehicle data.\n\n"
    "  --mass            kg, with battery (required)\n"
    "  --hover-throttle  normalized hover throttle from geo-tuner-hover / PX4 log"
    " (required)\n"
    "  --latency         estimated EKF+mavros+offboard latency [s]\n"
    "  --separation      attitude-BW / position-BW timescale ratio\n"
    "  --wn              requested position-loop wn [rad/s]; default = max allowed\n"
    "  --max-tilt        rad\n"
    "  --max-accel       m/s^2\n";
}

[[noreturn]] void fail(const char * prog, const std::string & msg)
{
  std::cerr << prog << ": error: " << msg << "\n";
  std::exit(2);
}

}  // namespace

int main(int argc, char ** argv)
{
  const char * prog = "geo-tuner-design";
  std::optional<double> mass, hover_throttle, wn;
  double attctrl_tau = 0.3, zeta = 0.95, latency = 0.08, separation = 4.0;
  double max_tilt = 0.52, max_accel = 5.0;
  std::string out_dir = ".";

  const std::map<std::string, double *> doubles{
    {"--attctrl-tau", &attctrl_tau}, {"--zeta", &zeta}, {"--latency", &latency},
    {"--separation", &separation}, {"--max-tilt", &max_tilt},
    {"--max-accel", &max_accel}};

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
        if (i + 1 >= argc) {fail(prog, "argument " + a + " expects a value");}
        return argv[++i];
      };
    if (a == "-h" || a == "--help") {
      usage(prog);
      return 0;
    } else if (a == "--mass") {
      mass = std::stod(next());
    } else if (a == "--hover-throttle") {
      hover_throttle = std::stod(next());
    } else if (a == "--wn") {
      wn = std::stod(next());
    } else if (a == "--out-dir") {
      out_dir = next();
    } else if (doubles.count(a)) {
      *doubles.at(a) = std::stod(next());
    } else {
      usage(prog);
      fail(prog, "unrecognized argument: " + a);
    }
  }

  if (!mass) {fail(prog, "the following argument is required: --mass");}
  if (!hover_throttle) {
    fail(prog, "the following argument is required: --hover-throttle");
  }
  if (!(*hover_throttle > 0.0 && *hover_throttle < 1.0)) {
    fail(prog, "--hover-throttle must be in (0, 1)");
  }

  geo_tuner::VehicleParams vehicle;
  vehicle.mass = *mass;
  vehicle.hover_throttle = *hover_throttle;
  vehicle.max_tilt_angle = max_tilt;

  geo_tuner::LoopShape shape;
  shape.attctrl_tau = attctrl_tau;
  shape.zeta = zeta;
  shape.latency = latency;
  shape.timescale_separation = separation;

  const auto g = geo_tuner::design_gains(vehicle, shape, wn);

  std::printf("=== Gain design summary ===\n");
  std::printf("mass            : %.3f kg\n", *mass);
  std::printf("hover throttle  : %.3f\n", *hover_throttle);
  std::printf("max_thrust      : %.2f N\n", g.max_thrust);
  std::printf(
    "accel headroom  : +%.1f m/s^2 vertical, %.1f m/s^2 lateral @ tilt limit\n",
    vehicle.vertical_accel_headroom(), vehicle.lateral_accel_max());
  std::printf(
    "attitude BW     : %.1f rad/s (attctrl_tau=%g)\n",
    shape.attitude_bandwidth(), attctrl_tau);
  std::printf(
    "position wn     : xy=%.2f  z=%.2f rad/s   zeta=%g\n", g.wn_xy, g.wn_z, g.zeta);
  std::printf("kx              : (%g, %g, %g)\n", g.kx[0], g.kx[1], g.kx[2]);
  std::printf("kv              : (%g, %g, %g)\n", g.kv[0], g.kv[1], g.kv[2]);
  for (const auto & n : g.notes) {
    std::printf("NOTE: %s\n", n.c_str());
  }

  std::error_code ec;
  std::filesystem::create_directories(out_dir, ec);
  const auto ctrl_path = std::filesystem::path(out_dir) / "geometric_controller.yaml";
  const auto mav_path = std::filesystem::path(out_dir) / "geometric_mavros.yaml";

  auto dump = [](const std::filesystem::path & path, const YAML::Node & node) {
      std::ofstream f(path);
      if (!f) {
        std::cerr << "could not write " << path << "\n";
        std::exit(1);
      }
      YAML::Emitter out;
      out << node;
      f << out.c_str() << "\n";
    };
  dump(
    ctrl_path,
    geo_tuner::geometric_controller_yaml(g, *mass, max_tilt, max_accel));
  dump(mav_path, geo_tuner::geometric_mavros_yaml(g));
  std::printf("\nwrote %s\nwrote %s\n", ctrl_path.c_str(), mav_path.c_str());
  return 0;
}
