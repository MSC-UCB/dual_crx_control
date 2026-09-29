// Small stateful bridge to the system Ruckig library; ROS remains in Python.
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <tuple>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <ruckig/ruckig.hpp>

using Arm = std::array<double, 6>;
using Joints = std::array<double, 12>;

class Generator {
public:
  Generator(double dt, const Arm &velocity, const Arm &acceleration, const Arm &jerk)
      : otg_(dt) {
    if (!std::isfinite(dt) || dt <= 0.) {
      throw std::invalid_argument("Ruckig timestep must be finite and positive");
    }
    input_.current_position.fill(0.);
    input_.current_velocity.fill(0.);
    input_.current_acceleration.fill(0.);
    input_.target_position.fill(0.);
    input_.target_velocity.fill(0.);
    input_.target_acceleration.fill(0.);
    input_.enabled.fill(false);
    input_.synchronization = ruckig::Synchronization::Time;
    for (size_t i = 0; i < 12; ++i) {
      for (double limit : {velocity[i % 6], acceleration[i % 6], jerk[i % 6]}) {
        if (!std::isfinite(limit) || limit <= 0.) {
          throw std::invalid_argument("Ruckig limits must be finite and positive");
        }
      }
      input_.max_velocity[i] = velocity[i % 6];
      input_.max_acceleration[i] = acceleration[i % 6];
      input_.max_jerk[i] = jerk[i % 6];
    }
  }

  void target(size_t arm, const Arm &q, const Arm &start, const Arm &velocity,
              const Arm &target_velocity, const Arm &target_acceleration) {
    if (arm > 1) {
      throw std::invalid_argument("Expected arm index 0 or 1");
    }
    for (size_t j = 0; j < 6; ++j) {
      if (!std::isfinite(q[j]) || !std::isfinite(start[j]) || !std::isfinite(velocity[j]) ||
          !std::isfinite(target_velocity[j]) || !std::isfinite(target_acceleration[j])) {
        throw std::invalid_argument("Ruckig joint state must be finite");
      }
    }
    for (size_t j = 0; j < 6; ++j) {
      const size_t i = arm * 6 + j;
      if (!input_.enabled[i]) {
        // Plan relative to the initial pose to avoid cancellation on tiny moves
        // around large absolute joint angles (notably near pi on the right arm).
        origins_[i] = start[j];
        input_.current_position[i] = 0.;
        input_.current_velocity[i] = velocity[j];
        input_.current_acceleration[i] = 0.;
        input_.enabled[i] = true;
      }
      // Only the target changes on subsequent input; preserve reference q/v/a.
      const double relative = q[j] - origins_[i];
      // Subtraction such as 1.02 - 1.0 leaves ~1e-17 rad noise that can
      // destabilize Ruckig's boundary-case root solver. Normalize only the
      // target displacement to 1e-12 rad; never reset/round reference v or a.
      input_.target_position[i] = std::abs(relative) < 1e6
          ? std::round(relative * 1e12) / 1e12 : relative;
      input_.target_velocity[i] = target_velocity[j];
      input_.target_acceleration[i] = target_acceleration[j];
    }
  }

  std::tuple<Joints, Joints, Joints> step() {
    const auto result = otg_.update(input_, output_);
    if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
      throw std::runtime_error("Ruckig update failed: " + std::to_string(static_cast<int>(result)));
    }
    for (size_t i = 0; i < 12; ++i) {
      if (!std::isfinite(output_.new_position[i]) || !std::isfinite(output_.new_velocity[i]) ||
          !std::isfinite(output_.new_acceleration[i])) {
        throw std::runtime_error("Ruckig returned a nonfinite reference state");
      }
    }
    output_.pass_to_input(input_);
    Joints position = output_.new_position;
    for (size_t i = 0; i < 12; ++i) {
      position[i] += origins_[i];
    }
    return {position, output_.new_velocity, output_.new_acceleration};
  }

private:
  ruckig::Ruckig<12> otg_;
  ruckig::InputParameter<12> input_;
  ruckig::OutputParameter<12> output_;
  Joints origins_ {};
};

PYBIND11_MODULE(_ruckig, module) {
  pybind11::class_<Generator>(module, "Generator")
      .def(pybind11::init<double, const Arm &, const Arm &, const Arm &>())
      .def("target", &Generator::target)
      .def("step", &Generator::step);
}
