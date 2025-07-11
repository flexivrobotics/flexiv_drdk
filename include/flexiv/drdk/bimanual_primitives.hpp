/**
 * @file bimanual_primitives.hpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_DRDK_BIMANUAL_PRIMITIVES_HPP_
#define FLEXIV_DRDK_BIMANUAL_PRIMITIVES_HPP_

#include "robot_pair.hpp"

namespace flexiv {
namespace drdk {

/**
 * @brief Which arm's TCP is used to describe a bimanual motion in Cartesian space.
 */
enum class ReferenceArm
{
    UNKOWN = 0, ///< Not set
    LEFT,       ///< Left arm, which is also the first robot in RobotPair
    RIGHT,      ///< Right arm, which is also the second robot in RobotPair
};

/**
 * @class BimanualPrimitives
 * @brief Interface to execute various bimanual manipulation primitives.
 */
class BimanualPrimitives
{
public:
    /**
     * @brief [Non-blocking] Instantiate the bimanual primitives interface.
     * @param[in] robot_pair Reference to the instance of flexiv::drdk::RobotPair.
     * @throw std::runtime_error if the initialization sequence failed.
     */
    BimanualPrimitives(const RobotPair& robot_pair);
    virtual ~BimanualPrimitives();

    /**
     * @brief [Blocking] Move two arms together in Cartesian space with tension or compression
     * applied between the end effectors of the two arms. While one arm tracks the target, the other
     * arm follows along and maintain the desired tension or compression.
     * @param[in] ref_arm Use this arm's TCP to follow the Cartesian target.
     * @param[in] target Target TCP pose of the reference arm w.r.t. the shared world frame: \f$
     * {^{O}T_{TCP}}_{d} \in \mathbb{R}^{7 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1}
     * \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion: \f$ [x, y, z, q_w, q_x, q_y,
     * q_z]^T \f$. Unit: \f$ [m]:[] \f$.
     * @param[in] tension_force Tension force between the two TCPs. Positive: stretch, negative:
     * compress. Unit: \f$ [N] \f$.
     * @param[in] linear_vel Linear Cartesian velocity when moving. Unit: \f$ [m/s] \f$.
     * @param[in] angular_vel Angular Cartesian velocity when moving. Unit: \f$ [rad/s] \f$.
     * @param[in] linear_acc Linear Cartesian acceleration when moving. A safe value is provided as
     * default. Unit: \f$ [m/s^2] \f$.
     * @param[in] angular_acc Angular Cartesian acceleration when moving. A safe value is provided
     * as default. Unit: \f$ [rad/s^2] \f$.
     */
    void MoveTensioned(ReferenceArm ref_arm, const std::array<double, kPoseSize>& target,
        double tension_force, double linear_vel, double angular_vel, double linear_acc = 2.0,
        double angular_acc = 5.0);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} /* namespace drdk */
} /* namespace flexiv */

#endif /* FLEXIV_DRDK_BIMANUAL_PRIMITIVES_HPP_ */
