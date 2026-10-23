/**
 * @example intermediate5_realtime_cartesian_pure_motion_control.cpp
 * This tutorial runs real-time Cartesian-space pure motion control to hold or sine-sweep the TCPs
 * of both robots. A simple collision detection is also included.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/drdk/robot_pair.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>

#include <iostream>
#include <string>
#include <cmath>
#include <limits>
#include <thread>
#include <atomic>

using namespace flexiv;

namespace {
/** RT loop frequency [Hz] */
constexpr size_t kLoopFreq = 1000;

/** RT loop period [sec] */
constexpr double kLoopPeriod = 0.001;

/** TCP sine-sweep amplitude [m] */
constexpr double kSwingAmp = 0.1;

/** TCP sine-sweep frequency [Hz] */
constexpr double kSwingFreq = 0.3;

/** External TCP force threshold for collision detection, value is only for demo purpose [N] */
constexpr double kExtForceThreshold = 10.0;

/** External joint torque threshold for collision detection, value is only for demo purpose [Nm] */
constexpr double kExtTorqueThreshold = 5.0;

/** Atomic signal to stop scheduler tasks */
std::atomic<bool> g_stop_sched = {false};
}

/** @brief Print program usage help */
void PrintHelp()
{
    // clang-format off
    std::cout << "Required arguments: [left_robot_sn] [right_robot_sn]" << std::endl;
    std::cout << "    left_robot_sn: Serial number of the left robot to connect. Remove any space, e.g. Rizon4s-100001" << std::endl;
    std::cout << "    right_robot_sn: Serial number of the right robot to connect. Remove any space, e.g. Rizon4s-100002" << std::endl;
    std::cout << "Optional arguments: [--hold] [--collision]" << std::endl;
    std::cout << "    --hold: robots hold current TCP pose, otherwise do a sine-sweep" << std::endl;
    std::cout << "    --collision: enable collision detection, robots will stop upon collision" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

/** @brief Simple collision detection based on the external force at TCP and the external joint
 * torques of a robot */
bool CollisionDetected(const rdk::RobotStates& states)
{
    Eigen::Vector3d ext_force = {states.ext_wrench_in_world[0], states.ext_wrench_in_world[1],
        states.ext_wrench_in_world[2]};
    if (ext_force.norm() > kExtForceThreshold) {
        return true;
    }
    for (const auto& v : states.tau_ext) {
        if (fabs(v) > kExtTorqueThreshold) {
            return true;
        }
    }
    return false;
}

/** @brief Callback function for realtime periodic task */
void PeriodicTask(drdk::RobotPair& robot_pair,
    const std::pair<std::array<double, rdk::kPoseSize>, std::array<double, rdk::kPoseSize>>&
        init_pose,
    const std::pair<std::vector<double>, std::vector<double>>& init_q, bool enable_hold,
    bool enable_collision)
{
    // Local periodic loop counter
    static uint64_t loop_counter = 0;

    try {
        // Monitor fault on the connected robots
        if (robot_pair.fault()) {
            throw std::runtime_error(
                "PeriodicTask: Fault occurred on the connected robots, exiting ...");
        }

        // Initialize target poses to initial poses
        auto target_pose = init_pose;

        // Sine-sweep TCPs of both robots along Y axis of the shared world frame
        if (!enable_hold) {
            double offset = kSwingAmp * sin(2 * M_PI * kSwingFreq * loop_counter * kLoopPeriod);
            target_pose.first[1] = init_pose.first[1] + offset;
            target_pose.second[1] = init_pose.second[1] + offset;
        }
        // Otherwise robot TCPs will hold at initial poses

        // Send commands. Calling this method with only target pose input results in pure motion
        // control
        robot_pair.StreamCartesianMotionForce(target_pose);

        // Do the following operations in sequence for every 20 seconds
        switch (loop_counter % (20 * kLoopFreq)) {
            // Online change reference joint positions at 3 seconds
            case (3 * kLoopFreq): {
                std::vector<double> preferred_jnt_pos
                    = {0.938, -1.108, -1.254, 1.464, 1.073, 0.278, -0.658};
                robot_pair.SetNullSpacePosture({preferred_jnt_pos, preferred_jnt_pos});
                std::cout << "Reference joint positions of both robots set to: "
                          << rdk::utility::Vec2Str(preferred_jnt_pos) << std::endl;
            } break;
            // Online change stiffness to half of nominal at 6 seconds
            case (6 * kLoopFreq): {
                auto info = robot_pair.info();
                auto new_K = std::make_pair(info.first.K_x_nom, info.second.K_x_nom);
                for (auto& v : new_K.first) {
                    v *= 0.5;
                }
                for (auto& v : new_K.second) {
                    v *= 0.5;
                }
                robot_pair.SetCartesianImpedance(new_K);
                std::cout << "Cartesian stiffness of the left robot set to: "
                          << rdk::utility::Arr2Str(new_K.first) << std::endl;
                std::cout << "Cartesian stiffness of the right robot set to: "
                          << rdk::utility::Arr2Str(new_K.second) << std::endl;
            } break;
            // Online change to another reference joint positions at 9 seconds
            case (9 * kLoopFreq): {
                std::vector<double> preferred_jnt_pos
                    = {-0.938, -1.108, 1.254, 1.464, -1.073, 0.278, 0.658};
                robot_pair.SetNullSpacePosture({preferred_jnt_pos, preferred_jnt_pos});
                std::cout << "Reference joint positions of both robots set to: "
                          << rdk::utility::Vec2Str(preferred_jnt_pos) << std::endl;
            } break;
            // Online reset impedance properties to nominal at 12 seconds
            case (12 * kLoopFreq): {
                auto info = robot_pair.info();
                robot_pair.SetCartesianImpedance({info.first.K_x_nom, info.second.K_x_nom});
                std::cout << "Cartesian impedance properties are reset" << std::endl;
            } break;
            // Online reset reference joint positions to nominal at 14 seconds
            case (14 * kLoopFreq): {
                robot_pair.SetNullSpacePosture(init_q);
                std::cout << "Reference joint positions are reset" << std::endl;
            } break;
            // Online enable max contact wrench regulation at 16 seconds
            case (16 * kLoopFreq): {
                std::array<double, rdk::kCartDoF> max_wrench = {10.0, 10.0, 10.0, 2.0, 2.0, 2.0};
                robot_pair.SetMaxContactWrench({max_wrench, max_wrench});
                std::cout << "Max contact wrench of both robots set to: "
                          << rdk::utility::Arr2Str(max_wrench) << std::endl;
            } break;
            // Disable max contact wrench regulation at 19 seconds
            case (19 * kLoopFreq): {
                std::array<double, rdk::kCartDoF> inf;
                inf.fill(std::numeric_limits<double>::infinity());
                robot_pair.SetMaxContactWrench({inf, inf});
                std::cout << "Max contact wrench regulation is disabled" << std::endl;
            } break;
            default:
                break;
        }

        // Simple collision detection: stop both robots if collision is detected from either
        // end-effector or robot body of either robot
        if (enable_collision) {
            auto states = robot_pair.states();
            if (CollisionDetected(states.first) || CollisionDetected(states.second)) {
                robot_pair.Stop();
                std::cerr << "[warn] Collision detected, stopping robots and exit program ..."
                          << std::endl;
                g_stop_sched = true;
            }
        }

        // Increment loop counter
        loop_counter++;

    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        g_stop_sched = true;
    }
}

int main(int argc, char* argv[])
{
    // Program Setup
    // =============================================================================================
    // Parse parameters
    if (argc < 3 || rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }

    // Serial numbers of the left and right robot respectively
    std::string left_robot_sn = argv[1];
    std::string right_robot_sn = argv[2];

    // Print description
    std::cout << ">>> Tutorial description <<<\nThis tutorial runs real-time Cartesian-space pure "
                 "motion control to hold or sine-sweep the TCPs of both robots. A simple collision "
                 "detection is also included.\n"
              << std::endl;

    // Type of motion specified by user
    bool enable_hold = false;
    if (rdk::utility::ProgramArgsExist(argc, argv, "--hold")) {
        std::cout << "Robots holding current TCP pose" << std::endl;
        enable_hold = true;
    } else {
        std::cout << "Robots running TCP sine-sweep" << std::endl;
    }

    // Whether to enable collision detection
    bool enable_collision = false;
    if (rdk::utility::ProgramArgsExist(argc, argv, "--collision")) {
        std::cout << "Collision detection enabled" << std::endl;
        enable_collision = true;
    } else {
        std::cout << "Collision detection disabled" << std::endl;
    }

    try {
        // DRDK Initialization
        // =========================================================================================
        // Instantiate robot pair interface
        drdk::RobotPair robot_pair({left_robot_sn, right_robot_sn});

        // Clear fault on the connected robots if any
        if (robot_pair.fault()) {
            std::cerr << "[warn] Fault occurred on one of the connected robots, trying to clear ..."
                      << std::endl;
            // Try to clear the fault for both robots
            auto result = robot_pair.ClearFault();
            // If fault is not cleared on both robots
            if (!(result.first && result.second)) {
                std::cerr << "[error] Fault cannot be cleared, exiting ..." << std::endl;
                return 1;
            }
            std::cout << "Fault on the connected robot is cleared" << std::endl;
        }

        // Enable the pair of robots, make sure the E-stop is released before enabling
        std::cout << "Enabling robots ..." << std::endl;
        robot_pair.Enable();

        // Wait for both robots to become operational
        while (!robot_pair.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "Both robots are now operational" << std::endl;

        // Move both robots to home pose
        std::cout << "Moving to home pose" << std::endl;
        robot_pair.SwitchMode(rdk::Mode::NRT_PLAN_EXECUTION);
        robot_pair.ExecutePlan({"PLAN-Home", "PLAN-Home"});
        // Wait for the plans to finish
        while (robot_pair.busy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // Zero Force-torque Sensors
        // =========================================================================================
        robot_pair.SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        // IMPORTANT: must zero force/torque sensor offset for accurate force/torque measurement
        robot_pair.ExecutePrimitive({"ZeroFTSensor", "ZeroFTSensor"}, {});

        // WARNING: during the process, the robots must not contact anything, otherwise the result
        // will be inaccurate and affect following operations
        std::cerr << "[warn] Zeroing force/torque sensors, make sure nothing is in contact with "
                     "the robots"
                  << std::endl;

        // Wait for the primitive to finish on both robots
        while (!std::get<int>(robot_pair.primitive_states().first["terminated"])
               || !std::get<int>(robot_pair.primitive_states().second["terminated"])) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "Sensor zeroing complete" << std::endl;

        // Configure Motion Control
        // =========================================================================================
        // The Cartesian motion force modes do pure motion control out of the box, thus nothing
        // needs to be explicitly configured

        // NOTE: motion control always uses the shared world frame, while force control can use
        // either world or TCP frame as reference frame

        // Start Pure Motion Control
        // =========================================================================================
        // Switch both robots to real-time mode for continuous motion control
        robot_pair.SwitchMode(rdk::Mode::RT_CARTESIAN_MOTION_FORCE);

        // Set all Cartesian axis(s) of both robots to motion control
        std::array<bool, rdk::kCartDoF> force_ctrl_axes
            = {false, false, false, false, false, false};
        robot_pair.SetForceControlAxis({force_ctrl_axes, force_ctrl_axes});

        // Save initial poses in the shared world frame and initial joint positions
        auto states = robot_pair.states();
        auto init_pose = std::make_pair(states.first.tcp_pose, states.second.tcp_pose);
        auto init_q = std::make_pair(states.first.q, states.second.q);

        // Create real-time scheduler to run periodic tasks
        rdk::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.AddTask(std::bind(PeriodicTask, std::ref(robot_pair), std::ref(init_pose),
                              std::ref(init_q), enable_hold, enable_collision),
            "HP periodic", 1, scheduler.max_priority());
        // Start all added tasks
        scheduler.Start();

        // Block and wait for signal to stop scheduler tasks
        while (!g_stop_sched) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        // Received signal to stop scheduler tasks
        scheduler.Stop();

    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
