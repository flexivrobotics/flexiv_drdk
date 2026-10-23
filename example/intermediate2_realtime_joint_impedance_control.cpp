/**
 * @example intermediate2_realtime_joint_impedance_control.cpp
 * This tutorial runs real-time joint impedance control to hold or sine-sweep all joints of both
 * robots.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/drdk/robot_pair.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>

#include <iostream>
#include <string>
#include <cmath>
#include <thread>
#include <atomic>

using namespace flexiv;

namespace {
/** RT loop period [sec] */
constexpr double kLoopPeriod = 0.001;

/** Sine-sweep trajectory amplitude and frequency */
constexpr double kSineAmp = 0.035;
constexpr double kSineFreq = 0.3;

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
    std::cout << "Optional arguments: [--hold]" << std::endl;
    std::cout << "    --hold: robots hold current joint positions, otherwise do a sine-sweep" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

/** @brief Callback function for realtime periodic task */
void PeriodicTask(drdk::RobotPair& robot_pair, const std::string& motion_type,
    const std::pair<std::vector<double>, std::vector<double>>& init_pos)
{
    // Local periodic loop counter
    static unsigned int loop_counter = 0;

    try {
        // Monitor fault on the connected robots
        if (robot_pair.fault()) {
            throw std::runtime_error(
                "PeriodicTask: Fault occurred on the connected robots, exiting ...");
        }

        // Initialize target vectors to hold position
        auto target_pos = init_pos;
        std::pair<std::vector<double>, std::vector<double>> target_vel
            = {std::vector<double>(init_pos.first.size()),
                std::vector<double>(init_pos.second.size())};
        auto target_acc = target_vel;

        // Set target vectors based on motion type
        if (motion_type == "sine-sweep") {
            double offset = kSineAmp * sin(2 * M_PI * kSineFreq * loop_counter * kLoopPeriod);
            for (size_t i = 0; i < target_pos.first.size(); ++i) {
                target_pos.first[i] = init_pos.first[i] + offset;
            }
            for (size_t i = 0; i < target_pos.second.size(); ++i) {
                target_pos.second[i] = init_pos.second[i] + offset;
            }
        } else if (motion_type != "hold") {
            throw std::invalid_argument(
                "PeriodicTask: Unknown motion type. Accepted motion types: hold, sine-sweep");
        }

        // Reduce stiffness of both robots to half of nominal values after 5 seconds
        if (loop_counter == 5000) {
            auto info = robot_pair.info();
            auto new_Kq = std::make_pair(info.first.K_q_nom, info.second.K_q_nom);
            for (auto& v : new_Kq.first) {
                v *= 0.5;
            }
            for (auto& v : new_Kq.second) {
                v *= 0.5;
            }
            robot_pair.SetJointImpedance(new_Kq);
            std::cout << "Joint stiffness of the left robot set to ["
                      << rdk::utility::Vec2Str(new_Kq.first) << "]" << std::endl;
            std::cout << "Joint stiffness of the right robot set to ["
                      << rdk::utility::Vec2Str(new_Kq.second) << "]" << std::endl;
        }

        // Reset impedance properties of both robots to nominal values after another 5 seconds
        if (loop_counter == 10000) {
            auto info = robot_pair.info();
            robot_pair.SetJointImpedance({info.first.K_q_nom, info.second.K_q_nom});
            std::cout << "Joint impedance properties are reset" << std::endl;
        }

        // Send commands
        robot_pair.StreamJointPosition(target_pos, target_vel, target_acc);

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
    std::cout << ">>> Tutorial description <<<\nThis tutorial runs real-time joint impedance "
                 "control to hold or sine-sweep all joints of both robots.\n"
              << std::endl;

    // Type of motion specified by user
    std::string motion_type = "";
    if (rdk::utility::ProgramArgsExist(argc, argv, "--hold")) {
        std::cout << "Robots holding current pose" << std::endl;
        motion_type = "hold";
    } else {
        std::cout << "Robots running joint sine-sweep" << std::endl;
        motion_type = "sine-sweep";
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

        // Real-time Joint Impedance Control
        // =========================================================================================
        // Switch both robots to real-time joint impedance control mode
        robot_pair.SwitchMode(rdk::Mode::RT_JOINT_IMPEDANCE);

        // Set initial joint positions
        auto states = robot_pair.states();
        auto init_pos = std::make_pair(states.first.q, states.second.q);
        std::cout << "Initial joint positions of the left robot set to: "
                  << rdk::utility::Vec2Str(init_pos.first) << std::endl;
        std::cout << "Initial joint positions of the right robot set to: "
                  << rdk::utility::Vec2Str(init_pos.second) << std::endl;

        // Create real-time scheduler to run periodic tasks
        rdk::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.AddTask(std::bind(PeriodicTask, std::ref(robot_pair), std::ref(motion_type),
                              std::ref(init_pos)),
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
