/**
 * @file self_collision_monitor.hpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_DRDK_SELF_COLLISION_MONITOR_HPP_
#define FLEXIV_DRDK_SELF_COLLISION_MONITOR_HPP_

#include "robot_pair.hpp"

namespace flexiv {
namespace drdk {

/**
 * @class SelfCollisionMonitor
 * @brief Monitor any imminent collision between the two robots in the pair. Both robots will be
 * stopped if they are about to collide, regardless of the current task and control mode.
 */
class SelfCollisionMonitor
{
public:
    /**
     * @brief [Non-blocking] Instantiate the self collision monitor.
     * @param[in] robot_pair Reference to the instance of flexiv::drdk::RobotPair.
     * @param[in] skipped_links Name of the link(s) to skip for collision check.
     * @throw std::runtime_error if the initialization sequence failed.
     */
    SelfCollisionMonitor(
        const RobotPair& robot_pair, const std::vector<std::string>& skipped_links = {});
    virtual ~SelfCollisionMonitor();

    /**
     * @brief [Non-blocking] Start the monitor, which will run in a background periodic loop with
     * the specified loop interval.
     * @param[in] loop_interval_ms Desired interval for the periodic loop of the monitor. Shorter
     * loop interval results in higher loop frequency (freq = 1000/interval), which means faster
     * detection of imminent collision. The default value is 10 ms (100 Hz). Unit: [millisecond].
     * @throw std::logic_error if the monitor is already started and running.
     * @note A warning will be printed if the desired loop interval cannot be fulfilled. In that
     * case, please increase the interval or use a more powerful computer.
     */
    void Start(unsigned int loop_interval_ms = 10);

    /**
     * @brief [Non-blocking] Stop the monitor.
     */
    void Stop();

    /**
     * @brief [Non-blocking] Set minimum distance allowed between the robot pair. The collision
     * monitor will trigger both robots to stop if any point on the left robot and any point on the
     * right robot come closer than this distance.
     * @param[in] dist Minimum distance, default to 0.05. Unit: [m].
     * @warning When setting this value, allow enough buffer for the robots to decelerate before
     * coming to a complete stop.
     */
    void SetMinDistance(double dist);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} /* namespace drdk */
} /* namespace flexiv */

#endif /* FLEXIV_DRDK_SELF_COLLISION_MONITOR_HPP_ */
