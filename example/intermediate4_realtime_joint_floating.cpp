/**
 * @example intermediate4_realtime_joint_floating.cpp
 * This tutorial runs real-time joint floating on both robots with gentle velocity damping, gravity
 * compensation, and soft protection against position limits. This example is ideal for verifying
 * the system's whole-loop real-timeliness, accuracy of the robot dynamics model, and joint torque
 * control performance. If everything works well, all joints should float smoothly.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/drdk/robot_pair.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>

#include <iostream>
#include <string>
#include <thread>
#include <atomic>

using namespace flexiv;

namespace {
/** Joint velocity damping gains for floating */
const std::vector<double> kFloatingDamping = {10.0, 10.0, 5.0, 5.0, 1.0, 1.0, 1.0};

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
    std::cout << "Optional arguments: None" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

/** @brief Callback function for realtime periodic task */
void PeriodicTask(drdk::RobotPair& robot_pair)
{
    try {
        // Monitor fault on the connected robots
        if (robot_pair.fault()) {
            throw std::runtime_error(
                "PeriodicTask: Fault occurred on the connected robots, exiting ...");
        }

        // Set 0 joint torques
        auto states = robot_pair.states();
        std::pair<std::vector<double>, std::vector<double>> target_torque
            = {std::vector<double>(states.first.q.size()),
                std::vector<double>(states.second.q.size())};

        // Add some velocity damping
        for (size_t i = 0; i < target_torque.first.size(); ++i) {
            target_torque.first[i] += -kFloatingDamping[i] * states.first.dtheta[i];
        }
        for (size_t i = 0; i < target_torque.second.size(); ++i) {
            target_torque.second[i] += -kFloatingDamping[i] * states.second.dtheta[i];
        }

        // Send target joint torques to both robots, enable gravity compensation and joint limits
        // soft protection
        robot_pair.StreamJointTorque(target_torque, {true, true}, {true, true});

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
    std::cout << ">>> Tutorial description <<<\nThis tutorial runs real-time joint floating on "
                 "both robots with gentle velocity damping, gravity compensation, and soft "
                 "protection against position limits. This example is ideal for verifying the "
                 "system's whole-loop real-timeliness, accuracy of the robot dynamics model, and "
                 "joint torque control performance. If everything works well, all joints should "
                 "float smoothly.\n"
              << std::endl;

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

        // Real-time Joint Floating
        // =========================================================================================
        // Switch both robots to real-time joint torque control mode
        robot_pair.SwitchMode(rdk::Mode::RT_JOINT_TORQUE);

        // Create real-time scheduler to run periodic tasks
        rdk::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.AddTask(std::bind(PeriodicTask, std::ref(robot_pair)), "HP periodic", 1,
            scheduler.max_priority());
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
