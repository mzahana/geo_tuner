// Unit tests for the episode scheduling logic that shortens a session.
//
// These exercise the pure decision functions of the tuning conductor (leg
// schedule, settle predicate, adaptive episode end, sequential bucket
// stop, step-amplitude validation) on a bare TuningSchedule -- no ROS
// context, no vehicle. Ported 1:1 from the package's original pytest suite.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "geo_tuner/tuning_schedule.hpp"

using namespace geo_tuner;  // NOLINT(build/namespaces)

namespace
{

OdomSample fake_odom(
  std::array<double, 3> pos, std::array<double, 3> vel, double yaw = 0.0,
  double wz = 0.0)
{
  OdomSample s;
  s.pos = pos;
  s.vel = vel;
  s.quat = {std::cos(0.5 * yaw), 0.0, 0.0, std::sin(0.5 * yaw)};
  s.body_rates = {0.0, 0.0, wz};
  return s;
}

/// A TuningSchedule with the same field values the Python fixture used.
TuningSchedule make_schedule()
{
  TuningSchedule c;
  c.bidirectional = true;
  c.step_sign = 1.0;
  c.leg_offset = 0.0;
  c.step_applied = 0.0;
  c.axes = {"x"};
  c.axis_idx = 0;
  c.setpoint = {0.0, 0.0, 3.0};
  c.setpoint_yaw = 0.0;
  c.settle_tol_pos = 0.06;
  c.settle_tol_vel = 0.10;
  c.settle_tol_yaw = 0.05;
  c.settle_quiet_time = 0.4;
  c.episode_quiet_time = 0.5;
  c.episode_settle_band = 0.04;
  c.episode_settle_floor = 0.0;
  c.settle_tol_frac = 0.15;
  c.step_size = 0.5;
  c.step_size_z = 0.4;
  c.yaw_step = 0.5;
  c.step_t0 = 0.0;
  c.rep = 0;
  c.min_episodes = 2;
  c.episodes_per_rung = 3;
  c.early_stop_spread = 1.15;
  c.odom = fake_odom({0.0, 0.0, 3.0}, {0.0, 0.0, 0.0});
  return c;
}

double record(TuningSchedule & c, const std::vector<double> & ys, double dt = 0.02)
{
  c.recording.clear();
  for (size_t i = 0; i < ys.size(); ++i) {
    c.recording.emplace_back(static_cast<double>(i) * dt, ys[i]);
  }
  return c.recording.back().first;
}

}  // namespace

// ------------------------------------------------------------- leg schedule

TEST(LegSchedule, BidirectionalWalksOutAndBack)
{
  // 0 -> +d -> 0 -> -d -> 0: every leg is a step of size d and the
  // excursion never exceeds d from the hover point.
  auto c = make_schedule();
  std::vector<double> legs;
  for (int i = 0; i < 6; ++i) {
    const double next = c.next_leg(0.5);
    legs.push_back(next - c.leg_offset);   // the applied step
    c.leg_offset = next;
    if (c.leg_offset == 0.0) {             // flip after returning
      c.step_sign *= -1.0;
    }
  }
  const std::vector<double> expect{0.5, -0.5, -0.5, 0.5, 0.5, -0.5};
  ASSERT_EQ(legs.size(), expect.size());
  for (size_t i = 0; i < legs.size(); ++i) {
    EXPECT_NEAR(legs[i], expect[i], 1e-12) << "leg " << i;
    EXPECT_NEAR(std::abs(legs[i]), 0.5, 1e-12);
  }
}

TEST(LegSchedule, NeverWalksAwayFromTheHoverPoint)
{
  // Safety property: the commanded offset is always one step from the
  // hover point and returns to it -- steps never accumulate, over any
  // number of episodes or axis changes.
  auto c = make_schedule();
  const double d = 0.5;
  int seen_centre = 0;
  for (int n = 0; n < 40; ++n) {
    const double next = c.next_leg(d);
    ASSERT_LE(std::abs(next), d + 1e-12) << "episode " << n << ": offset " << next;
    c.leg_offset = next;
    if (c.leg_offset == 0.0) {
      ++seen_centre;
      c.step_sign *= -1.0;
    }
  }
  // back at centre every other episode, and the walk is balanced
  EXPECT_EQ(seen_centre, 20);
  EXPECT_TRUE(c.leg_offset == 0.0 || c.leg_offset == d || c.leg_offset == -d);
}

TEST(LegSchedule, AxisChangeRecentres)
{
  // advance_axis clears the leg schedule, so a new axis always starts its
  // steps from the hover point.
  auto c = make_schedule();
  c.leg_offset = 0.5;
  c.setpoint_yaw = 0.5;
  c.axes = {"x", "y"};
  const std::array<double, 3> hover{0.0, 0.0, 3.0};
  // Recentring returns to the hover point AND the heading the session was
  // handed over at -- not to yaw 0, which would snap the vehicle to north.
  const double hover_yaw = 1.3;
  EXPECT_FALSE(c.advance_axis(hover, hover_yaw));   // x -> y, no wrap
  EXPECT_EQ(c.leg_offset, 0.0);
  EXPECT_EQ(c.setpoint_yaw, hover_yaw);
  EXPECT_EQ(c.setpoint, hover);
  EXPECT_TRUE(c.advance_axis(hover, hover_yaw));    // y -> wrap back to x
  EXPECT_EQ(c.axis_idx, 0u);
}

TEST(ZLegClearance, UpLegAlwaysClears)
{
  // The guard is about flying at the ground; upward legs never do, whatever
  // the altitude.
  EXPECT_TRUE(z_leg_clears_floor(1.0, 0.4, 0.4, 0.5, 2.0));
  EXPECT_TRUE(z_leg_clears_floor(1.0, 0.0, 0.4, 0.5, 2.0));
}

TEST(ZLegClearance, DownLegCountsTheStepAndItsOvershoot)
{
  // 5 m AGL, 0.4 m step, 50% overshoot allowance -> bottom of the manoeuvre
  // is 5 - 0.4 - 0.2 = 4.4 m. (Tested either side of the boundary, not on
  // it: 5.0 - 0.4 - 0.2 is 4.3999999999999995 in binary floating point.)
  EXPECT_TRUE(z_leg_clears_floor(5.0, -0.4, 0.4, 0.5, 4.39));
  EXPECT_FALSE(z_leg_clears_floor(5.0, -0.4, 0.4, 0.5, 4.41));
  // The same altitude and floor, but a bigger vertical step (a live
  // parameter), no longer clears: 5 - 1.0 - 0.5 = 3.5.
  EXPECT_TRUE(z_leg_clears_floor(5.0, -1.0, 1.0, 0.5, 3.49));
  EXPECT_FALSE(z_leg_clears_floor(5.0, -1.0, 1.0, 0.5, 3.51));
  // The field case that started this: judged on odometry z (-3.8) rather
  // than AGL (2.8), every one of these answers is wrong.
  EXPECT_TRUE(z_leg_clears_floor(2.8, -0.4, 0.4, 0.5, 2.0));
  EXPECT_FALSE(z_leg_clears_floor(-3.8, -0.4, 0.4, 0.5, 2.0));
}

TEST(Ramp, NeverOvershootsAndStopsExactly)
{
  EXPECT_DOUBLE_EQ(ramp_toward(0.0, 10.0, 0.1), 0.1);
  EXPECT_DOUBLE_EQ(ramp_toward(0.0, -10.0, 0.1), -0.1);
  // Inside one step, land exactly on the target (no limit cycle).
  EXPECT_DOUBLE_EQ(ramp_toward(9.95, 10.0, 0.1), 10.0);
  // A zero rate means "no ramp": jump. Callers opt in deliberately.
  EXPECT_DOUBLE_EQ(ramp_toward(0.0, 10.0, 0.0), 10.0);
}

TEST(Ramp, VectorMovesAlongTheLineOfTravel)
{
  const std::array<double, 3> from{0.0, 0.0, 0.0};
  const std::array<double, 3> to{3.0, 4.0, 0.0};   // 5 m away
  const auto next = ramp_toward(from, to, 1.0);
  EXPECT_NEAR(next[0], 0.6, 1e-9);
  EXPECT_NEAR(next[1], 0.8, 1e-9);
  EXPECT_NEAR(next[2], 0.0, 1e-9);
  // ...and 5 such steps arrive, without overshoot
  auto p = from;
  for (int i = 0; i < 5; ++i) {p = ramp_toward(p, to, 1.0);}
  EXPECT_NEAR(p[0], to[0], 1e-9);
  EXPECT_NEAR(p[1], to[1], 1e-9);
}

TEST(LegSchedule, ClassicScheduleAlwaysStepsOut)
{
  auto c = make_schedule();
  c.bidirectional = false;
  EXPECT_NEAR(c.next_leg(0.5), 0.5, 1e-12);
  c.step_sign = -1.0;
  EXPECT_NEAR(c.next_leg(0.5), -0.5, 1e-12);
}

// --------------------------------------------------------- settle predicate

TEST(SettlePredicate, RequiresQuietForTheFullWindow)
{
  auto c = make_schedule();
  EXPECT_FALSE(c.is_quiet(0.0));   // first quiet sample only starts it
  EXPECT_FALSE(c.is_quiet(0.3));
  EXPECT_TRUE(c.is_quiet(0.45));
}

TEST(SettlePredicate, MotionResetsTheWindow)
{
  auto c = make_schedule();
  c.is_quiet(0.0);
  c.odom = fake_odom({0.0, 0.0, 3.0}, {0.5, 0.0, 0.0});   // still moving
  EXPECT_FALSE(c.is_quiet(0.3));
  c.odom = fake_odom({0.0, 0.0, 3.0}, {0.0, 0.0, 0.0});
  EXPECT_FALSE(c.is_quiet(0.5));   // window restarted at 0.5
  EXPECT_TRUE(c.is_quiet(0.95));
}

TEST(SettlePredicate, PositionErrorBlocksQuiet)
{
  auto c = make_schedule();
  c.odom = fake_odom({0.4, 0.0, 3.0}, {0.0, 0.0, 0.0});
  EXPECT_FALSE(c.is_quiet(0.0));
  EXPECT_FALSE(c.is_quiet(2.0));
}

TEST(SettlePredicate, YawAxisAlsoChecksHeading)
{
  auto c = make_schedule();
  c.axes = {"yaw"};
  c.setpoint_yaw = 0.5;
  c.odom = fake_odom({0.0, 0.0, 3.0}, {0.0, 0.0, 0.0}, 0.0);
  EXPECT_FALSE(c.is_quiet(1.0));   // heading not there yet
  c.odom = fake_odom({0.0, 0.0, 3.0}, {0.0, 0.0, 0.0}, 0.5);
  c.is_quiet(1.0);
  EXPECT_TRUE(c.is_quiet(1.5));
}

// ----------------------------------------------------- adaptive episode end

TEST(AdaptiveEpisodeEnd, SettledResponseEndsTheEpisode)
{
  auto c = make_schedule();
  c.step_applied = 0.5;
  const double t_end = record(c, std::vector<double>(200, 0.5));
  EXPECT_TRUE(c.response_settled(t_end));
}

TEST(AdaptiveEpisodeEnd, StillMovingDoesNot)
{
  auto c = make_schedule();
  c.step_applied = 0.5;
  std::vector<double> ys;
  for (int i = 0; i < 200; ++i) {
    ys.push_back(0.5 + 0.05 * std::sin(4.0 * i * 0.02));
  }
  const double t_end = record(c, ys);
  EXPECT_FALSE(c.response_settled(t_end));
}

TEST(AdaptiveEpisodeEnd, FlatTransportDelayIsNotMistakenForSettling)
{
  // Right after the step the response is flat at ~0 for the delay; that
  // window must not end the episode.
  auto c = make_schedule();
  c.step_applied = 0.5;
  const double t_end = record(c, std::vector<double>(30, 0.0));
  EXPECT_FALSE(c.response_settled(t_end));
}

TEST(AdaptiveEpisodeEnd, WrongDirectionRejected)
{
  auto c = make_schedule();
  c.step_applied = -0.5;
  const double t_end = record(c, std::vector<double>(200, 0.5));
  EXPECT_FALSE(c.response_settled(t_end));
}

// -------------------------------------------------------- sequential stop

TEST(SequentialStop, StopsEarlyOnTightAgreement)
{
  auto c = make_schedule();
  c.rep = 2;
  c.bucket.add(1.20, 0.1);
  c.bucket.add(1.25, 0.1);
  EXPECT_TRUE(c.bucket_settled());
}

TEST(SequentialStop, NotBeforeTheMinimum)
{
  auto c = make_schedule();
  c.rep = 1;
  c.bucket.add(1.20, 0.1);
  EXPECT_FALSE(c.bucket_settled());
}

TEST(SequentialStop, DisagreementFliesTheFullBucket)
{
  auto c = make_schedule();
  c.rep = 2;
  c.bucket.add(1.00, 0.1);
  c.bucket.add(1.30, 0.1);   // 1.30x > early_stop_spread 1.15x
  EXPECT_FALSE(c.bucket_settled());
}

TEST(SequentialStop, ADiscardedEpisodeBlocksTheEarlyStop)
{
  // Two flown, one accepted: the identification is already noisy, so the
  // bucket must run to full length.
  auto c = make_schedule();
  c.rep = 2;
  c.bucket.add(1.20, 0.1);
  EXPECT_FALSE(c.bucket_settled());
}

TEST(SequentialStop, EarlyStopGateIsStricterThanTheUpdateGate)
{
  // An early stop must never let through evidence that the full bucket's
  // consistency gate would reject.
  auto c = make_schedule();
  EXPECT_LE(c.early_stop_spread, 1.35);
}

// ---------------------------------------------------------- step amplitude
// The commanded envelope is hover +/- the configured amplitude, and the
// amplitude is validated against what the safety monitor allows.

TEST(StepAmplitude, AxisStepMagnitudeFollowsTheAxis)
{
  auto c = make_schedule();
  c.axes = {"x", "z", "yaw"};
  c.axis_idx = 0;
  EXPECT_NEAR(c.axis_step_mag(), 0.5, 1e-12);
  c.axis_idx = 1;
  EXPECT_NEAR(c.axis_step_mag(), 0.4, 1e-12);
  c.axis_idx = 2;
  EXPECT_NEAR(c.axis_step_mag(), 0.5, 1e-12);
}

TEST(StepAmplitude, EnvelopeIsTheAmplitude)
{
  // A 0.2 m amplitude confines the manoeuvre to +/-0.2 m.
  auto c = make_schedule();
  c.step_size = 0.2;
  double worst = 0.0;
  for (int i = 0; i < 8; ++i) {
    c.leg_offset = c.next_leg(c.axis_step_mag());
    worst = std::max(worst, std::abs(c.leg_offset));
    if (c.leg_offset == 0.0) {c.step_sign *= -1.0;}
  }
  EXPECT_NEAR(worst, 0.2, 1e-12);
}

TEST(StepAmplitude, QuietToleranceTightensWithASmallStep)
{
  // A 0.06 m absolute tolerance is 30% of a 0.2 m step; the fractional
  // gate must take over.
  auto c = make_schedule();
  c.step_size = 0.2;
  c.odom = fake_odom({0.05, 0.0, 3.0}, {0.0, 0.0, 0.0});
  EXPECT_FALSE(c.is_quiet(0.0));   // 0.05 m > 0.15 * 0.2 = 0.03 m
  EXPECT_FALSE(c.is_quiet(1.0));
  c.odom = fake_odom({0.02, 0.0, 3.0}, {0.0, 0.0, 0.0});
  c.is_quiet(1.0);
  EXPECT_TRUE(c.is_quiet(1.5));
}

TEST(StepAmplitude, SettleBandHasANoiseFloor)
{
  // A tiny step must not demand a flatness finer than the odometry noise,
  // which would silently disable the adaptive stop.
  auto c = make_schedule();
  c.step_applied = 0.1;
  c.episode_settle_floor = 0.01;
  std::vector<double> ys;
  for (int i = 0; i < 200; ++i) {ys.push_back(0.1 + (i % 2 ? 0.004 : -0.004));}
  const double t_end = record(c, ys);       // 8 mm ripple
  EXPECT_TRUE(c.response_settled(t_end));   // under the 10 mm floor
  c.episode_settle_floor = 0.0;             // band = 0.04 * 0.1 = 4 mm
  EXPECT_FALSE(c.response_settled(t_end));
}

// ------------------------------------------------------ amplitude validation

namespace
{

TuningSchedule amplitude_validator(double max_step = 2.0)
{
  auto c = make_schedule();
  c.max_step_size = max_step;
  c.min_step_size = 0.05;
  c.max_yaw_step = 1.0;
  c.small_step_warn = 0.25;
  return c;
}

}  // namespace

TEST(AmplitudeValidation, StepMustStayUnderThePositionErrorAbort)
{
  // A step commands its full amplitude as instantaneous position error,
  // so a step at max_pos_error aborts the session on the spot.
  auto c = amplitude_validator();
  EXPECT_NEAR(c.step_ceiling(2.0), 1.6, 1e-12);   // 0.8 * 2.0
  EXPECT_TRUE(c.validate_step(1.5, false, 2.0).first);
  const auto [ok, why] = c.validate_step(1.9, false, 2.0);
  EXPECT_FALSE(ok);
  EXPECT_NE(why.find("max_pos_error"), std::string::npos);
}

TEST(AmplitudeValidation, MaxStepSizeCanBeTheBindingLimit)
{
  auto c = amplitude_validator(1.0);
  EXPECT_NEAR(c.step_ceiling(10.0), 1.0, 1e-12);
  EXPECT_FALSE(c.validate_step(1.2, false, 10.0).first);
}

TEST(AmplitudeValidation, ConfinedSpaceAmplitudesAcceptedWithAWarning)
{
  auto c = amplitude_validator();
  auto [ok, note] = c.validate_step(0.15, false, 2.0);
  EXPECT_TRUE(ok);
  EXPECT_NE(note.find("nrmse"), std::string::npos);   // allowed, but flagged as small
  std::tie(ok, note) = c.validate_step(0.5, false, 2.0);
  EXPECT_TRUE(ok);
  EXPECT_EQ(note, "");
}

TEST(AmplitudeValidation, DegenerateValuesRejected)
{
  auto c = amplitude_validator();
  EXPECT_FALSE(c.validate_step(0.0, false, 2.0).first);
  EXPECT_FALSE(c.validate_step(-0.5, false, 2.0).first);
  EXPECT_FALSE(c.validate_step(std::nan(""), false, 2.0).first);
}

TEST(AmplitudeValidation, YawStepBounds)
{
  auto c = amplitude_validator();
  EXPECT_TRUE(c.validate_step(0.5, true, 2.0).first);
  EXPECT_FALSE(c.validate_step(1.5, true, 2.0).first);
  EXPECT_FALSE(c.validate_step(0.01, true, 2.0).first);
}

// ---- yaw-final-rung eligibility and session progress ----

TEST(AxisEligibility, YawFliesOnlyTheFinalRung)
{
  EXPECT_FALSE(axis_eligible("yaw", 0, 2, true));
  EXPECT_TRUE(axis_eligible("yaw", 1, 2, true));
  EXPECT_TRUE(axis_eligible("yaw", 0, 1, true));   // single rung IS the final rung
  for (size_t r = 0; r < 3; ++r) {
    EXPECT_TRUE(axis_eligible("x", r, 3, true));
    EXPECT_TRUE(axis_eligible("z", r, 3, true));
  }
}

TEST(AxisEligibility, DisabledMeansEveryRung)
{
  EXPECT_TRUE(axis_eligible("yaw", 0, 2, false));
  EXPECT_TRUE(axis_eligible("yaw", 1, 2, false));
}

TEST(SessionProgress, CountsOnlyEligibleBuckets)
{
  const std::vector<std::string> axes{"z", "x", "y", "yaw"};
  // 2 rungs x 4 axes x 3 eps = 24 without the skip; yaw on rung 0 skipped -> 21.
  auto [done0, total] = session_progress(axes, 2, 3, 0, 0, 0, true);
  EXPECT_EQ(total, 21);
  EXPECT_EQ(done0, 0);
  // Mid-session: rung 1, axis y (index 2), one episode flown on it. Behind
  // us: rung 0 (z, x, y = 9) + rung 1 z, x (6) = 15, plus the one flown.
  auto [done1, total1] = session_progress(axes, 2, 3, 1, 2, 1, true);
  EXPECT_EQ(total1, 21);
  EXPECT_EQ(done1, 16);
}

TEST(SessionProgress, MonotoneAcrossTheWholeSession)
{
  const std::vector<std::string> axes{"z", "x", "y", "yaw"};
  int last = -1;
  for (size_t r = 0; r < 2; ++r) {
    for (size_t a = 0; a < axes.size(); ++a) {
      for (int rep = 0; rep <= 3; ++rep) {
        auto [done, total] = session_progress(axes, 2, 3, r, a, rep, true);
        EXPECT_GE(done, last);
        EXPECT_LE(done, total);
        last = done;
      }
    }
  }
  EXPECT_EQ(last, 21);
}
