// Flight legs flown through Px4Link, one at a time, stepped at the control rate:
//   Engage   stream a setpoint above the current position, take offboard and arm, climb to z.
//   Vertical climb or descend to z over a fixed point (PX4 limits the rate, MPC_Z_VEL_MAX_*).
//   Line     fly a horizontal segment at fixed z: a carrot moves along it with a trapezoid speed
//            profile (cruise speed, accel limit) and its velocity is fed forward, so the vehicle
//            holds the commanded cruise speed instead of PX4's MPC_XY_VEL_MAX.
//   Hold     keep streaming a setpoint at a point; never finishes. Ends a leg list that must
//            leave the vehicle in offboard (holding at a fix, waiting for a marker lock).
//   Precland hand over to PX4 precision land and wait until landed and disarmed.
//   FixedWing VTOL only (Phase 7): transition to fixed wing, follow the line from the start to
//   the target at the target's z (PX4 offboard path setpoint: the point and the track
//   velocity), and transition back to multicopter once within back_transition_m of the target;
//   done when PX4 flies as a multicopter again.
// A leg fails if PX4 leaves the mode the leg needs; the caller then stops commanding.
#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <optional>
#include <string>

#include "dd_flight/px4_link.hpp"

namespace dd_flight
{

struct Leg
{
  enum class Kind {kEngage, kVertical, kLine, kHold, kPrecland, kFixedWing} kind;
  Vec3 target{};          // NED; Line uses x, y and z; Vertical uses x, y, z.
  double speed_mps = 0;   // Line and FixedWing.
  double back_transition_m = 0;  // FixedWing only.
};

class Maneuver
{
public:
  enum class Status {kIdle, kRunning, kDone, kFailed};

  struct Limits
  {
    double accel_mps2;
    double arrive_radius_m;
    double settle_speed_mps;
    double engage_timeout_s;
    double transition_timeout_s = 40.0;
  };

  Maneuver(Px4Link & px4, Limits limits)
  : px4_(px4), lim_(limits) {}

  void start(std::deque<Leg> legs, double yaw)
  {
    legs_ = std::move(legs);
    yaw_ = yaw;
    status_ = legs_.empty() ? Status::kDone : Status::kRunning;
    begin_leg();
  }

  [[nodiscard]] Status status() const {return status_;}
  [[nodiscard]] const std::string & failure() const {return failure_;}
  // Kind of the leg being flown; none when idle, finished or failed.
  [[nodiscard]] std::optional<Leg::Kind> leg() const
  {
    if (status_ != Status::kRunning) {
      return std::nullopt;
    }
    return legs_.front().kind;
  }
  [[nodiscard]] bool in_precland_leg() const
  {
    return status_ == Status::kRunning && legs_.front().kind == Leg::Kind::kPrecland;
  }

  void step(double dt)
  {
    if (status_ != Status::kRunning) {
      return;
    }
    leg_time_ += dt;
    const Leg & leg = legs_.front();
    bool done = false;
    switch (leg.kind) {
      case Leg::Kind::kEngage: done = step_engage(leg); break;
      case Leg::Kind::kVertical: done = step_vertical(leg); break;
      case Leg::Kind::kLine: done = step_line(leg, dt); break;
      case Leg::Kind::kHold:
        px4_.setpoint(leg.target, {kNan, kNan, kNan}, yaw_);
        require_offboard();
        break;
      case Leg::Kind::kPrecland: done = step_precland(); break;
      case Leg::Kind::kFixedWing: done = step_fixed_wing(leg); break;
    }
    if (status_ == Status::kFailed) {
      return;
    }
    if (done) {
      legs_.pop_front();
      if (legs_.empty()) {
        status_ = Status::kDone;
      } else {
        begin_leg();
      }
    }
  }

private:
  static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
  // Offboard needs setpoints flowing before PX4 accepts the switch.
  static constexpr double kPrimeS = 1.0;
  static constexpr double kRequestPeriodS = 0.5;
  static constexpr double kZTolM = 0.3;

  void begin_leg()
  {
    leg_time_ = 0.0;
    last_request_ = -1e9;
    if (status_ != Status::kRunning) {
      return;
    }
    const Leg & leg = legs_.front();
    start_ = px4_.position();
    phase_ = FwPhase::kToFixedWing;
    phase_start_ = 0.0;
    if (leg.kind == Leg::Kind::kLine || leg.kind == Leg::Kind::kFixedWing) {
      start_[2] = leg.target[2];
      const double dx = leg.target[0] - start_[0], dy = leg.target[1] - start_[1];
      length_ = std::hypot(dx, dy);
      dir_ = length_ > 0 ? Vec3{dx / length_, dy / length_, 0} : Vec3{0, 0, 0};
      s_ = 0.0;
      v_ = 0.0;
    }
  }

  void fail(const std::string & why)
  {
    status_ = Status::kFailed;
    failure_ = why;
  }

  [[nodiscard]] bool settled(const Vec3 & target) const
  {
    const Vec3 & p = px4_.position();
    const Vec3 & v = px4_.velocity();
    return std::hypot(p[0] - target[0], p[1] - target[1]) < lim_.arrive_radius_m &&
           std::abs(p[2] - target[2]) < kZTolM &&
           std::hypot(v[0], v[1], v[2]) < lim_.settle_speed_mps;
  }

  bool step_engage(const Leg & leg)
  {
    px4_.setpoint(leg.target, {kNan, kNan, kNan}, yaw_);
    if (px4_.offboard() && px4_.armed()) {
      return settled(leg.target);
    }
    if (leg_time_ > lim_.engage_timeout_s) {
      fail("PX4 did not accept offboard and arming");
    } else if (leg_time_ > kPrimeS && leg_time_ - last_request_ > kRequestPeriodS) {
      last_request_ = leg_time_;
      if (!px4_.offboard()) {
        px4_.request_offboard();
      } else {
        px4_.arm();
      }
    }
    return false;
  }

  bool require_offboard()
  {
    if (!px4_.offboard() || !px4_.armed()) {
      fail("PX4 left offboard");
      return false;
    }
    return true;
  }

  bool step_vertical(const Leg & leg)
  {
    px4_.setpoint(leg.target, {kNan, kNan, kNan}, yaw_);
    return require_offboard() && settled(leg.target);
  }

  bool step_line(const Leg & leg, double dt)
  {
    const double remaining = length_ - s_;
    v_ = std::min({leg.speed_mps, v_ + lim_.accel_mps2 * dt,
      std::sqrt(2.0 * lim_.accel_mps2 * std::max(remaining, 0.0))});
    s_ = std::min(length_, s_ + v_ * dt);
    const Vec3 carrot{start_[0] + dir_[0] * s_, start_[1] + dir_[1] * s_, leg.target[2]};
    px4_.setpoint(carrot, {dir_[0] * v_, dir_[1] * v_, kNan}, yaw_);
    return require_offboard() && s_ >= length_ && settled(leg.target);
  }

  bool step_precland()
  {
    if (!px4_.armed() && px4_.landed() && leg_time_ > kPrimeS) {
      return true;
    }
    if (!px4_.precland()) {
      if (px4_.armed() && leg_time_ - last_request_ > kRequestPeriodS) {
        last_request_ = leg_time_;
        px4_.request_precland();
      }
      // Keep offboard fed until PX4 has switched, so it never falls back to hold first.
      if (px4_.offboard()) {
        px4_.setpoint(start_, {kNan, kNan, kNan}, yaw_);
      }
      if (leg_time_ > lim_.engage_timeout_s && px4_.armed()) {
        fail("PX4 did not enter precision land");
      }
    }
    return false;
  }

  enum class FwPhase {kToFixedWing, kCruise, kToMulticopter};

  bool step_fixed_wing(const Leg & leg)
  {
    const Vec3 & p = px4_.position();
    const double to_go = std::hypot(leg.target[0] - p[0], leg.target[1] - p[1]);
    if (!require_offboard()) {return false;}
    if (phase_ == FwPhase::kToFixedWing || phase_ == FwPhase::kCruise) {
      // Path setpoint: the target with the track velocity, so PX4 follows the lane line.
      px4_.setpoint(leg.target, {dir_[0] * leg.speed_mps, dir_[1] * leg.speed_mps, 0.0}, yaw_);
    } else {
      px4_.setpoint(leg.target, {kNan, kNan, kNan}, yaw_);
    }
    switch (phase_) {
      case FwPhase::kToFixedWing:
        if (px4_.fixed_wing()) {
          next_phase(FwPhase::kCruise);
        } else if (!px4_.in_transition()) {
          request_transition(true);
        }
        break;
      case FwPhase::kCruise:
        if (to_go < leg.back_transition_m) {next_phase(FwPhase::kToMulticopter);}
        break;
      case FwPhase::kToMulticopter:
        if (px4_.multicopter()) {return true;}
        if (!px4_.in_transition()) {request_transition(false);}
        break;
    }
    if (phase_ != FwPhase::kCruise && leg_time_ - phase_start_ > lim_.transition_timeout_s) {
      fail(phase_ == FwPhase::kToFixedWing ? "no transition to fixed wing" :
        "no transition back to multicopter");
    }
    return false;
  }

  void next_phase(FwPhase to)
  {
    phase_ = to;
    phase_start_ = leg_time_;
    last_request_ = -1e9;
  }

  void request_transition(bool to_fixed_wing)
  {
    if (leg_time_ - last_request_ > kRequestPeriodS) {
      last_request_ = leg_time_;
      px4_.transition(to_fixed_wing);
    }
  }

  Px4Link & px4_;
  Limits lim_;
  FwPhase phase_ = FwPhase::kToFixedWing;
  double phase_start_ = 0.0;
  std::deque<Leg> legs_;
  Status status_ = Status::kIdle;
  std::string failure_;
  double yaw_ = 0.0, leg_time_ = 0.0, last_request_ = 0.0;
  Vec3 start_{}, dir_{};
  double length_ = 0.0, s_ = 0.0, v_ = 0.0;
};

}  // namespace dd_flight
