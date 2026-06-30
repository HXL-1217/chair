// // lib/DualSteeredWheelchair.h
// #ifndef DUAL_STEERED_WHEELCHAIR_H
// #define DUAL_STEERED_WHEELCHAIR_H

// #include <cmath>
// #include <stdexcept>
// #include <limits>
// #include <algorithm> // for std::clamp

// #ifndef M_PI
//     #define M_PI 3.14159265358979323846
// #endif

// struct WheelCommands {
//     double vl;
//     double vr;
//     double thl;  // ∈ [−π, π], with −π and π physically distinct & non-wrapping
//     double thr;
// };

// class DualSteeredWheelchair {
// public:
//     explicit DualSteeredWheelchair(double wheel_track, double velocity_epsilon = 1e-4, double max_angle_step = 0.2)
//         : l_(wheel_track), eps_(velocity_epsilon), max_angle_step_(max_angle_step), last_thl_(0.0), last_thr_(0.0) {
//         if (l_ <= 0.0) {
//             throw std::invalid_argument("Wheel track must be positive.");
//         }
//         if (eps_ < 0.0) {
//             throw std::invalid_argument("Velocity epsilon must be non-negative.");
//         }
//         if (max_angle_step_ <= 0.0 || max_angle_step_ > M_PI) {
//             throw std::invalid_argument("Max angle step must be in (0, π].");
//         }
//     }

//     WheelCommands computeWheelCommands(double vx, double vy, double omega) {
//         const double vlx = vx - (l_ * 0.5) * omega;
//         const double vly = vy;
//         const double vrx = vx + (l_ * 0.5) * omega;
//         const double vry = vy;

//         const double vl_raw = std::hypot(vlx, vly);
//         const double vr_raw = std::hypot(vrx, vry);
//         const double thl_raw = std::atan2(vly, vlx);
//         const double thr_raw = std::atan2(vry, vrx);

//         double vl, vr, thl, thr;

//         if (vl_raw < eps_) {
//             vl = 0.0;
//             thl = last_thl_;
//         } else {
//             selectMinimalAbsoluteDeviation(thl_raw, vl_raw, last_thl_, thl, vl);
//             last_thl_ = thl;
//         }

//         if (vr_raw < eps_) {
//             vr = 0.0;
//             thr = last_thr_;
//         } else {
//             selectMinimalAbsoluteDeviation(thr_raw, vr_raw, last_thr_, thr, vr);
//             last_thr_ = thr;
//         }

//         return {vl, vr, thl, thr};
//     }
    
//     // 横向移动（+y 方向）
//     void resetToLateral() {
//         last_thl_ = clampAngleStep(last_thl_, M_PI / 2.0);   // left wheel: +90°
//         last_thr_ = clampAngleStep(last_thr_, -M_PI / 2.0);  // right wheel: -90°
//     }

//     void resetSteeringAngles(double thl = 0.0, double thr = 0.0) {
//         thl = std::clamp(thl, -M_PI, M_PI);
//         thr = std::clamp(thr, -M_PI, M_PI);
//         last_thl_ = normalizeAngle(thl);
//         last_thr_ = normalizeAngle(thr);
//     }

//     double getWheelTrack() const { return l_; }
//     double getVelocityEpsilon() const { return eps_; }
    
//     // std::pair<double, double> getCurrentAngles() const {
//     //     return {last_thl_, last_thr_};
//     // }

//     struct BodyVelocity {
//         double vx;
//         double vy;
//         double omega;
//     };

//     //  修正后的正运动学（逆时针 ω > 0）
//     BodyVelocity computeForwardKinematics(double vl, double vr, double thl, double thr) const {
//         const double cos_thl = std::cos(thl);
//         const double sin_thl = std::sin(thl);
//         const double cos_thr = std::cos(thr);
//         const double sin_thr = std::sin(thr);

//         const double vx = 0.5 * (vl * cos_thl + vr * cos_thr);
//         const double vy = 0.5 * (vl * sin_thl + vr * sin_thr);
//         const double omega = (vr * cos_thr - vl * cos_thl) / l_;  // ← 关键修正！

//         return {vx, vy, omega};
//     }

// private:
//     static constexpr double PI = M_PI;
//     static constexpr double NEG_PI = -M_PI;
    
//     static double normalizeAngle(double angle) {
//         while (angle > PI) angle -= 2 * PI;
//         while (angle <= NEG_PI) angle += 2 * PI;
//         return angle;
//     }
    
//     double clampAngleStep(double current, double target) const {
//         double diff = normalizeAngle(target - current);
//         if (std::abs(diff) > max_angle_step_) {
//             diff = std::copysign(max_angle_step_, diff);
//         }
//         return normalizeAngle(current + diff);
//     }

//     void selectMinimalAbsoluteDeviation(
//         double raw_theta, double raw_v,
//         double last_theta,
//         double& out_theta, double& out_v) const
//     {
//         // 候选解：(θ, v), (θ+π, -v), (θ-π, -v)
//         struct Candidate {
//             double theta, v;
//         } candidates[3];
//         int ncand = 0;

//         candidates[ncand++] = {raw_theta, raw_v};
//         candidates[ncand++] = {raw_theta + PI, -raw_v};
//         candidates[ncand++] = {raw_theta - PI, -raw_v};

//         double best_dev = std::numeric_limits<double>::max();
//         int best_idx = 0;

//         for (int i = 0; i < ncand; ++i) {
//             double dev = std::abs(normalizeAngle(candidates[i].theta - last_theta));
//             if (dev < best_dev) {
//                 best_dev = dev;
//                 best_idx = i;
//             }
//         }

//         out_theta = normalizeAngle(candidates[best_idx].theta);
//         out_v = candidates[best_idx].v;
//     }

//     const double l_;
//     const double eps_;
//     const double max_angle_step_;
//     double last_thl_;
//     double last_thr_;
// };

// #endif // DUAL_STEERED_WHEELCHAIR_H




// lib/DualSteeredWheelchair.h
#ifndef DUAL_STEERED_WHEELCHAIR_H
#define DUAL_STEERED_WHEELCHAIR_H

#include <cmath>
#include <stdexcept>
#include <limits>
#include <algorithm>

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

struct WheelCommands {
  double vl;
  double vr;
  double thl;  // ∈ [-π, π], -π 与 +π 物理上不同，不允许绕圈
  double thr;
};

class DualSteeredWheelchair {
public:
  explicit DualSteeredWheelchair(double wheel_track,
                                 double velocity_epsilon = 1e-4,
                                 double max_angle_step = 0.2)
  : l_(wheel_track),
    eps_(velocity_epsilon),
    max_angle_step_(max_angle_step),
    last_thl_(0.0),
    last_thr_(0.0)
  {
    if (l_ <= 0.0) throw std::invalid_argument("Wheel track must be positive.");
    if (eps_ < 0.0) throw std::invalid_argument("Velocity epsilon must be non-negative.");
    if (max_angle_step_ <= 0.0 || max_angle_step_ > M_PI)
      throw std::invalid_argument("Max angle step must be in (0, π].");
  }

  WheelCommands computeWheelCommands(double vx, double vy, double omega) {
    const double vlx = vx - (l_ * 0.5) * omega;
    const double vly = vy;
    const double vrx = vx + (l_ * 0.5) * omega;
    const double vry = vy;

    const double vl_raw  = std::hypot(vlx, vly);
    const double vr_raw  = std::hypot(vrx, vry);
    const double thl_raw = std::atan2(vly, vlx);
    const double thr_raw = std::atan2(vry, vrx);

    double vl, vr, thl, thr;

    if (vl_raw < eps_) {
      vl  = 0.0;
      thl = last_thl_;
    } else {
      selectClosestNonWrapping(thl_raw, vl_raw, last_thl_, thl, vl);
      last_thl_ = thl;
    }

    if (vr_raw < eps_) {
      vr  = 0.0;
      thr = last_thr_;
    } else {
      selectClosestNonWrapping(thr_raw, vr_raw, last_thr_, thr, vr);
      last_thr_ = thr;
    }

    return {vl, vr, thl, thr};
  }

  // 横向移动（+y 方向）
  void resetToLateral() {
    last_thl_ = clampAngleStepNonWrap(last_thl_,  M_PI / 2.0);   // +90°
    last_thr_ = clampAngleStepNonWrap(last_thr_, -M_PI / 2.0);   // -90°
  }

  void resetSteeringAngles(double thl = 0.0, double thr = 0.0) {
    thl = std::clamp(thl, -M_PI, M_PI);
    thr = std::clamp(thr, -M_PI, M_PI);
    last_thl_ = clampToServoRange(thl);
    last_thr_ = clampToServoRange(thr);
  }

  double getWheelTrack() const { return l_; }
  double getVelocityEpsilon() const { return eps_; }

  struct BodyVelocity {
    double vx;
    double vy;
    double omega;
  };

  BodyVelocity computeForwardKinematics(double vl, double vr, double thl, double thr) const {
    const double cos_thl = std::cos(thl);
    const double sin_thl = std::sin(thl);
    const double cos_thr = std::cos(thr);
    const double sin_thr = std::sin(thr);

    const double vx = 0.5 * (vl * cos_thl + vr * cos_thr);
    const double vy = 0.5 * (vl * sin_thl + vr * sin_thr);
    const double omega = (vr * cos_thr - vl * cos_thl) / l_;

    return {vx, vy, omega};
  }

private:
  static constexpr double PI = M_PI;

  // 把角度映射到 [-π, π]，注意：不把 -π 折叠成 +π（两者物理不同）
  static double clampToServoRange(double a) {
    while (a >  PI) a -= 2.0 * PI;
    while (a < -PI) a += 2.0 * PI;
    // a 现在在 [-π, π]，-π 仍保留为 -π
    return a;
  }

  // “非绕圈”的步进限制：舵机不允许跨 ±π 走捷径
  double clampAngleStepNonWrap(double current, double target) const {
    current = std::clamp(current, -PI, PI);
    target  = std::clamp(target,  -PI, PI);

    double diff = (target - current);                 // 不做 normalizeAngle
    if (std::abs(diff) > max_angle_step_) {
      diff = std::copysign(max_angle_step_, diff);
    }

    double out = current + diff;
    return std::clamp(out, -PI, PI);
  }

  // 选择离 last_theta 最近的等价解（非绕圈距离）
  // 候选解： (θ, +v), (θ+π, -v), (θ-π, -v)
  void selectClosestNonWrapping(double raw_theta, double raw_v,
                                double last_theta,
                                double& out_theta, double& out_v) const
  {
    struct Candidate { double theta; double v; };

    Candidate cand[3] = {
      {raw_theta,       raw_v},
      {raw_theta + PI, -raw_v},
      {raw_theta - PI, -raw_v}
    };

    // last_theta 也保证在 [-π, π]
    last_theta = std::clamp(last_theta, -PI, PI);

    double best_dev = std::numeric_limits<double>::max();
    int best_idx = 0;

    for (int i = 0; i < 3; ++i) {
      const double th = clampToServoRange(cand[i].theta); // 候选映射进舵机范围
      const double dev = std::abs(th - last_theta);       // 非绕圈距离（关键！）

      if (dev < best_dev) {
        best_dev = dev;
        best_idx = i;
      }
    }

    out_theta = clampToServoRange(cand[best_idx].theta);
    out_v     = cand[best_idx].v;
  }

  const double l_;
  const double eps_;
  const double max_angle_step_;
  double last_thl_;
  double last_thr_;
};

#endif // DUAL_STEERED_WHEELCHAIR_H
