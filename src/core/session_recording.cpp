#include "geo_tuner/core/session_recording.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace geo_tuner
{

std::vector<AxisSegment> SessionRecording::segments(int axis, size_t first) const
{
  std::vector<AxisSegment> out;
  int cur = -1;
  for (size_t i = first; i < samples.size(); ++i) {
    const auto & s = samples[i];
    if (out.empty() || s.seg != cur) {
      out.emplace_back();
      cur = s.seg;
    }
    auto & g = out.back();
    g.r.push_back(s.r[axis]);
    g.v.push_back(s.v[axis]);
    g.u.push_back(s.u[axis]);
    g.kx.push_back(s.kx[axis]);
    g.kv.push_back(s.kv[axis]);
  }
  return out;
}

const char * SessionRecording::csv_header()
{
  return "t,seg,rx,ry,rz,vx,vy,vz,ux,uy,uz,kxx,kxy,kxz,kvx,kvy,kvz";
}

std::string SessionRecording::csv_line(const SessionSample & s)
{
  char buf[512];
  std::snprintf(
    buf, sizeof(buf),
    "%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
    s.t, s.seg, s.r[0], s.r[1], s.r[2], s.v[0], s.v[1], s.v[2], s.u[0], s.u[1], s.u[2],
    s.kx[0], s.kx[1], s.kx[2], s.kv[0], s.kv[1], s.kv[2]);
  return buf;
}

bool SessionRecording::save_csv(const std::string & path) const
{
  std::ofstream f(path);
  if (!f) {return false;}
  f << csv_header() << "\n";
  for (const auto & s : samples) {f << csv_line(s) << "\n";}
  return static_cast<bool>(f);
}

SessionRecording SessionRecording::load_csv(const std::string & path)
{
  std::ifstream f(path);
  if (!f) {throw std::runtime_error("cannot open " + path);}
  SessionRecording rec;
  std::string line;
  size_t lineno = 0;
  while (std::getline(f, line)) {
    ++lineno;
    if (line.empty() || line[0] == '#' || line[0] == 't') {continue;}
    std::stringstream ss(line);
    std::string cell;
    std::vector<double> v;
    while (std::getline(ss, cell, ',')) {v.push_back(std::stod(cell));}
    if (v.size() != 17) {
      throw std::runtime_error(
        path + ":" + std::to_string(lineno) + ": expected 17 columns, got " +
        std::to_string(v.size()));
    }
    SessionSample s;
    s.t = v[0];
    s.seg = static_cast<int>(v[1]);
    for (int a = 0; a < 3; ++a) {
      s.r[a] = v[2 + a];
      s.v[a] = v[5 + a];
      s.u[a] = v[8 + a];
      s.kx[a] = v[11 + a];
      s.kv[a] = v[14 + a];
    }
    rec.samples.push_back(s);
  }
  return rec;
}

}  // namespace geo_tuner
