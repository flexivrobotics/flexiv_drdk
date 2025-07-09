/**
 * @example basics1_display_robot_states.cpp
 * This tutorial does the very first thing: check connection with a pair of robots and print the
 * received robot states.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/drdk/robot_pair.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>

#include <iostream>
#include <thread>

using namespace flexiv;

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

/** @brief Print robot states data @ 1Hz */
void PrintRobotStates(drdk::RobotPair& robot_pair)
{
    while (true) {
        // Print all robot states in JSON format using the built-in ostream operator overloading
        spdlog::info("Left robot states:");
        std::cout << robot_pair.states().first << std::endl;
        spdlog::info("Right robot states:");
        std::cout << robot_pair.states().second << std::endl;

        // Print digital inputs
        spdlog::info("Left robot digital inputs:");
        std::cout << rdk::utility::Arr2Str(robot_pair.digital_inputs().first) << std::endl;
        spdlog::info("Right robot digital inputs:");
        std::cout << rdk::utility::Arr2Str(robot_pair.digital_inputs().second) << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
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
    spdlog::info(
        ">>> Tutorial description <<<\nThis tutorial does the very first thing: check connection "
        "with a pair of robots and print received robot states.\n");

    try {
        // RDK Initialization
        // =========================================================================================
        // Instantiate robot pair interface
        drdk::RobotPair robot_pair({left_robot_sn, right_robot_sn});

        // Clear fault on the connected robots if any
        if (robot_pair.fault()) {
            spdlog::warn("Fault occurred on one of the connected robots, trying to clear ...");
            // Try to clear the fault
            auto result = robot_pair.ClearFault();
            if (!result.first || !result.second) {
                spdlog::error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            spdlog::info("Fault on the connected robot is cleared");
        }

        // Enable the pair of robots, make sure the E-stop is released before enabling
        spdlog::info("Enabling robots ...");
        robot_pair.Enable();

        // Wait for both robots to become operational
        while (!robot_pair.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        spdlog::info("Both robots are now operational");

        // Print States
        // =========================================================================================
        // Use std::thread to do scheduling so that this example can run on all OS, since not all OS
        // support rdk::Scheduler
        std::thread low_priority_thread(std::bind(PrintRobotStates, std::ref(robot_pair)));

        // Properly exit thread
        low_priority_thread.join();
    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
