#pragma once

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace stage_c {

struct IkPolicy {
  double position_residual = .003;
  double rotation_residual = .08;
  double max_joint_step = .35;
  double limit_epsilon = .001;  // At most 0.0573 degrees; never extends output limits.
  unsigned iterations = 200;
};

// No ROS, vendor library or hardware access. All returned joints obey hard limits.
class SeededIk {
  KDL::Chain chain_;
  std::vector<double> low_, high_;
  KDL::Frame tcp_;
  IkPolicy policy_;

  static std::string jointError(unsigned i, double q, double low, double high) {
    std::ostringstream s;
    s << "joint" << i + 1 << "=" << q << " rad outside [" << low << "," << high << "]";
    return s.str();
  }

  void seedValid(const std::vector<double>& seed) const {
    if (seed.size() != low_.size()) throw std::runtime_error("IK seed size mismatch");
    for (unsigned i = 0; i < seed.size(); ++i)
      if (!std::isfinite(seed[i]) || seed[i] < low_[i] || seed[i] > high_[i])
        throw std::runtime_error("invalid measured seed: " + jointError(i, seed[i], low_[i], high_[i]));
  }

public:
  SeededIk(const KDL::Chain& chain, const std::vector<double>& low,
           const std::vector<double>& high, const KDL::Vector& tcp, IkPolicy policy)
      : chain_(chain), low_(low), high_(high), tcp_(tcp), policy_(policy) {
    if (low.empty() || low.size() != high.size() || chain.getNrOfJoints() != low.size())
      throw std::runtime_error("IK chain/limits size mismatch");
    for (unsigned i = 0; i < low.size(); ++i)
      if (!std::isfinite(low[i]) || !std::isfinite(high[i]) || low[i] >= high[i])
        throw std::runtime_error("invalid IK limits");
    for (unsigned i = 0; i < 3; ++i)
      if (!std::isfinite(tcp[i])) throw std::runtime_error("nonfinite TCP offset");
    if (!std::isfinite(policy.position_residual) || policy.position_residual <= 0 ||
        !std::isfinite(policy.rotation_residual) || policy.rotation_residual <= 0 ||
        !std::isfinite(policy.max_joint_step) || policy.max_joint_step <= 0 ||
        !std::isfinite(policy.limit_epsilon) || policy.limit_epsilon < 0 ||
        policy.limit_epsilon > .001 || policy.iterations == 0 || policy.iterations > 1000)
      throw std::runtime_error("invalid IK policy");
  }

  KDL::Frame forward(const std::vector<double>& q) const {
    seedValid(q);
    KDL::JntArray joints(q.size());
    for (unsigned i = 0; i < q.size(); ++i) joints(i) = q[i];
    KDL::Frame f;
    KDL::ChainFkSolverPos_recursive fk(chain_);
    if (fk.JntToCart(joints, f) < 0) throw std::runtime_error("FK failed");
    return f * tcp_;
  }

  std::vector<double> validateCandidate(const KDL::Frame& desired,
      const std::vector<double>& seed, std::vector<double> candidate) const {
    seedValid(seed);
    if (candidate.size() != low_.size()) throw std::runtime_error("IK candidate size mismatch");
    for (unsigned i = 0; i < candidate.size(); ++i) {
      double q = candidate[i];
      if (!std::isfinite(q) || q < low_[i] - policy_.limit_epsilon ||
          q > high_[i] + policy_.limit_epsilon)
        throw std::runtime_error("IK limit: " + jointError(i, q, low_[i], high_[i]));
      candidate[i] = std::max(low_[i], std::min(high_[i], q));
      if (std::abs(candidate[i] - seed[i]) > policy_.max_joint_step) {
        // Subtraction at a solver bound may add a few ULPs (field: -0.35).
        // Project numerical noise inward; do not enlarge the continuity limit.
        if (std::abs(candidate[i] - seed[i]) <= policy_.max_joint_step + 1e-12) {
          while (std::abs(candidate[i] - seed[i]) > policy_.max_joint_step)
            candidate[i] = std::nextafter(candidate[i], seed[i]);
          continue;
        }
        std::ostringstream s;
        s << "IK continuity: joint" << i + 1 << " delta=" << candidate[i] - seed[i]
          << " rad exceeds " << policy_.max_joint_step;
        throw std::runtime_error(s.str());
      }
    }
    const auto f = forward(candidate);
    const double pos = (f.p - desired.p).Norm();
    const double rot = KDL::diff(f.M, desired.M).Norm();
    if (!std::isfinite(pos) || !std::isfinite(rot) ||
        pos > policy_.position_residual || rot > policy_.rotation_residual) {
      std::ostringstream s;
      s << "IK FK residual: position=" << pos << " m, rotation=" << rot << " rad";
      throw std::runtime_error(s.str());
    }
    return candidate;
  }

  std::vector<double> solve(const KDL::Frame& desired, const std::vector<double>& seed) const {
    seedValid(seed);
    for (unsigned i = 0; i < 3; ++i) {
      if (!std::isfinite(desired.p[i])) throw std::runtime_error("nonfinite target position");
      for (unsigned j = 0; j < 3; ++j)
        if (!std::isfinite(desired.M(i, j))) throw std::runtime_error("nonfinite target rotation");
    }
    KDL::JntArray initial(seed.size()), low(seed.size()), high(seed.size()), output(seed.size());
    for (unsigned i = 0; i < seed.size(); ++i) {
      initial(i) = output(i) = seed[i];
      low(i) = std::max(low_[i], seed[i] - policy_.max_joint_step);
      high(i) = std::min(high_[i], seed[i] + policy_.max_joint_step);
    }
    const KDL::Frame wrist = desired * tcp_.Inverse();
    KDL::ChainFkSolverPos_recursive fk(chain_);
    KDL::ChainIkSolverVel_pinv velocity(chain_);
    KDL::ChainIkSolverPos_NR_JL bounded(chain_, low, high, fk, velocity, policy_.iterations, 1e-7);
    const int bounded_code = bounded.CartToJnt(initial, wrist, output);
    std::string bounded_reason = "solver code " + std::to_string(bounded_code);
    auto accept = [&]() {
      std::vector<double> q(seed.size());
      for (unsigned i = 0; i < q.size(); ++i) q[i] = output(i);
      return validateCandidate(desired, seed, q);
    };
    if (bounded_code >= 0 || bounded_code == KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED) {
      try { return accept(); }
      catch (const std::exception& e) { bounded_reason = e.what(); }
    }
    // A damped, measured-seed fallback handles singular neighborhoods. Its output
    // passes the same limit, continuity and post-projection FK checks.
    KDL::ChainIkSolverPos_LMA damped(chain_, 1e-7, policy_.iterations);
    output = initial;
    const int damped_code = damped.CartToJnt(initial, wrist, output);
    if (damped_code >= 0 || damped_code == KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED ||
        damped_code == KDL::ChainIkSolverPos_LMA::E_GRADIENT_JOINTS_TOO_SMALL ||
        damped_code == KDL::ChainIkSolverPos_LMA::E_INCREMENT_JOINTS_TOO_SMALL) {
      try { return accept(); }
      catch (const std::exception& e) {
        throw std::runtime_error("bounded IK: " + bounded_reason + "; seeded IK: " + e.what());
      }
    }
    throw std::runtime_error("bounded IK: " + bounded_reason + "; seeded IK solver code " + std::to_string(damped_code));
  }
};
}  // namespace stage_c
