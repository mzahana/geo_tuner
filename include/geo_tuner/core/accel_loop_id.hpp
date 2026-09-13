// Identification of the acceleration loop the position controller closes.
//
// The geometric controller commands an acceleration a_cmd (it publishes it,
// times mass, as SE3Command.force). Everything below the position loop --
// thrust map, attitude loop, PX4 rate loop, EKF, transport -- is what turns
// that command into the acceleration the vehicle actually has. Modelled as
//
//     a(s) = alpha * e^(-d s) / (tau s + 1) * a_cmd(s) + w(s)          (1)
//
// alpha  plant-gain factor (1.0 = the controller's model is exact)
// d, tau pure delay and first-order lag of everything in the loop
// w      disturbance: wind, thrust ripple, unmodelled aero
//
// Why this and not step-response shape fitting
// --------------------------------------------
// A position step response at fixed gains measures one time scale. alpha
// and the lag both change that time scale, so one short record cannot
// separate them, and a gust over the 2-3 s transient moves the answer by
// the same amount a real change would. Flown evidence (2026-09-10/13, three
// flights, 35 fits): per-episode alpha scattered +/-12 % and the fitted
// lag 5-268 ms on one airframe; a Monte Carlo of that fitter with the
// measured wind (0.13 m/s^2, ~2 s correlation) and the TRUE alpha held at
// 1.0 reproduces both the scatter and the "lag on the solver floor" fits.
// Equation (1) instead uses every sample of the session, and the command
// and response are both measured, so gain and lag enter separately.
//
// Why an instrumental variable
// ----------------------------
// Under feedback, a_cmd reacts to w (the controller fights the gust), so
// ordinary least squares of a on a_cmd is biased -- on the flight data it
// read alpha 0.4-0.7. The setpoint r is chosen by the tuner and cannot
// react to wind, so it is a valid instrument: z = the acceleration the
// NOMINAL loop (alpha 1, no lag, the gains actually flown) would command
// for r. alpha_IV = <z, a> / <z, x> with x = a_cmd passed through the
// candidate delay and lag. The (d, tau) pair is chosen on a grid by the
// residual on the samples where the instrument is excited.
//
// Uncertainty
// -----------
// Delete-one-block jackknife over contiguous time blocks (the disturbance
// is correlated over seconds, so samples are not independent; blocks are).
// Intervals use Student t with n_blocks - 1 degrees of freedom. These are
// the intervals the decision logic acts on -- no spread gates.
#ifndef GEO_TUNER__CORE__ACCEL_LOOP_ID_HPP_
#define GEO_TUNER__CORE__ACCEL_LOOP_ID_HPP_

#include <string>
#include <vector>

namespace geo_tuner
{

/// One axis of the recording, on a UNIFORM time grid, split into contiguous
/// segments (a pause, e.g. leaving OFFBOARD, starts a new segment -- filters
/// are never run across a gap).
struct AxisSegment
{
  std::vector<double> r;      // commanded position (setpoint) [m or rad]
  std::vector<double> v;      // measured velocity [m/s]
  std::vector<double> u;      // commanded acceleration, gravity removed [m/s^2]
  std::vector<double> kx;     // position gain in force at each sample
  std::vector<double> kv;     // velocity gain in force at each sample
};

struct AccelLoopConfig
{
  double fs{50.0};              // sample rate of the segments [Hz]
  double lpf_hz{3.0};           // zero-phase low-pass on both sides [Hz]
  double delay_max{0.30};       // grid [s]
  double lag_max{0.30};         // grid [s]
  double grid_step{0.02};       // [s]; one sample at 50 Hz
  double excite_frac{0.15};     // |z| above this fraction of max counts as excited
  int n_blocks{8};              // jackknife blocks
  double min_excited_s{1.5};    // refuse with less excited data than this
  double min_r2{0.6};           // refuse a model that explains less than this
  double confidence{0.90};      // two-sided interval level (0.80, 0.90 or 0.95)
};

struct AccelLoopResult
{
  bool ok{false};
  std::string reason;           // why not ok
  double alpha{0.0};
  double alpha_lo{0.0}, alpha_hi{0.0};
  double delay{0.0};            // [s]
  double tau{0.0};              // [s]
  double lag{0.0};              // delay + tau [s], the number that costs phase
  double lag_lo{0.0}, lag_hi{0.0};
  double r2{0.0};               // of the setpoint-explained acceleration (gates min_r2)
  double r2_raw{0.0};           // of the raw excited acceleration (falls with wind)
  double excited_s{0.0};        // seconds of excited data used
  int n_samples{0};
};

AccelLoopResult identify_accel_loop(
  const std::vector<AxisSegment> & segments, const AccelLoopConfig & cfg = {});

// ---- building blocks, exposed for tests ----

/// Zero-phase 2nd-order Butterworth low-pass (forward-backward).
std::vector<double> filtfilt_lowpass(const std::vector<double> & x, double fc, double fs);

/// Acceleration the nominal loop (alpha 1, no lag) commands for setpoint r
/// with per-sample gains: p'' = kx (r - p) - kv p', returned value p''.
std::vector<double> nominal_command(
  const std::vector<double> & r, const std::vector<double> & kx,
  const std::vector<double> & kv, double fs);

/// Two-sided Student t quantile for the given confidence and dof.
double t_quantile(double confidence, int dof);

}  // namespace geo_tuner

#endif  // GEO_TUNER__CORE__ACCEL_LOOP_ID_HPP_
