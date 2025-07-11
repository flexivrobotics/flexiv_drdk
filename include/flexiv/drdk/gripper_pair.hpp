/**
 * @file gripper_pair.hpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_DRDK_GRIPPER_PAIR_HPP_
#define FLEXIV_DRDK_GRIPPER_PAIR_HPP_

#include "robot_pair.hpp"
#include <flexiv/rdk/gripper.hpp>

namespace flexiv {
namespace drdk {

using namespace rdk;

/**
 * @class GripperPair
 * @brief Interface to simultaneously control a pair of grippers installed respectively on the pair
 * of robots.
 */
class GripperPair
{
public:
    /**
     * @brief [Non-blocking] Instantiate the control interface of the gripper pair.
     * @param[in] robot_pair Reference to the instance of flexiv::drdk::RobotPair.
     * @throw std::runtime_error if the initialization sequence failed.
     */
    GripperPair(const RobotPair& robot_pair);
    virtual ~GripperPair();

    /**
     * @brief [Blocking] Enable the specified grippers as robot devices.
     * @param[in] names Respective names of the grippers to enable. To enable only one gripper,
     * specify its name and leave the other one empty.
     * @throw std::invalid_argument if either of the specified grippers does not exist.
     * @throw std::logic_error if a gripper is already enabled on either robot in the pair.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair or
     * failed to sync gripper parameters.
     * @note This function blocks until the request is successfully delivered.
     * @note There can only be one enabled gripper at a time per robot, call Disable() on the
     * currently enabled gripper before enabling another gripper.
     * @warning There's no enforced check on whether the enabled device is a gripper or not. Using
     * this function to enable a non-gripper device will likely lead to undefined behaviors.
     */
    void Enable(const std::pair<std::string, std::string>& names);

    /**
     * @brief [Blocking] Disable one or both of the currently enabled grippers.
     * @param[in] mask True: disable this gripper; false: skip this gripper.
     * @throw std::logic_error if no gripper is enabled.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note This function blocks until the request is successfully delivered.
     */
    void Disable(std::pair<bool, bool> mask);

    /**
     * @brief [Blocking] Manually trigger the initialization of the enabled grippers. This step is
     * not needed for grippers that automatically initialize upon power-on.
     * @throw std::logic_error if no gripper is enabled.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note This function blocks until the request is successfully delivered.
     * @warning This function does not wait for the initialization sequence to finish, the user may
     * need to implement wait after calling this function before commanding the gripper.
     */
    void Init();

    /**
     * @brief [Blocking] For both grippers in the pair, grasp with direct force control. This
     * function requires the enabled grippers to support direct force control.
     * @param[in] forces Respective target gripping forces. Positive: closing force, negative:
     * opening force. Valid range: [GripperParams::min_force, GripperParams::max_force].
     * Unit: \f$ [N] \f$.
     * @throw std::invalid_argument if either element in [forces] is outside the valid range.
     * @throw std::logic_error if no gripper is enabled.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note This function blocks until the request is successfully delivered.
     * @warning Target inputs outside the valid range (see params()) will be saturated.
     */
    void Grasp(std::pair<double, double> forces);

    /**
     * @brief [Blocking] For both grippers in the pair, move the fingers with position control.
     * @param[in] widths Respective target opening widths. Valid range: [GripperParams::min_width,
     * GripperParams::max_width]. Unit: \f$ [m] \f$.
     * @param[in] velocities Respective closing/opening velocities, cannot be 0. Valid range:
     * [GripperParams::min_vel, GripperParams::max_vel]. Unit: \f$ [m/s] \f$.
     * @param[in] force_limits Maximum contact force during movement. Valid range:
     * [GripperParams::min_force, GripperParams::max_force]. Unit: \f$ [N] \f$.
     * @throw std::invalid_argument if any input parameter is outside its valid range.
     * @throw std::logic_error if no gripper is enabled.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note This function blocks until the request is successfully delivered.
     * @warning Target inputs outside the valid range (see params()) will be saturated.
     */
    void Move(std::pair<double, double> widths, std::pair<double, double> velocities,
        std::pair<double, double> force_limits);

    /**
     * @brief [Blocking] Stop one or both grippers and hold their current finger widths.
     * @param[in] mask True: stop this gripper; false: skip this gripper.
     * @throw std::logic_error if no gripper is enabled.
     * @throw std::runtime_error if failed to deliver the request to the connected robot pair.
     * @note This function blocks until the request is successfully delivered.
     */
    void Stop(std::pair<bool, bool> mask);

    /**
     * @overload Stop both grippers.
     */
    void Stop();

    /**
     * @brief [Non-blocking] Parameters of the currently enabled grippers.
     * @return Respective value copies of rdk::GripperParams struct, empty if the corresponding
     * gripper is not enabled.
     */
    std::pair<GripperParams, GripperParams> params() const;

    /**
     * @brief [Non-blocking] Current states data of the enabled grippers.
     * @return Respective value copies of rdk::GripperStates struct, empty if the corresponding
     * gripper is not enabled.
     */
    std::pair<GripperStates, GripperStates> states() const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} /* namespace drdk */
} /* namespace flexiv */

#endif /* FLEXIV_DRDK_GRIPPER_PAIR_HPP_ */
