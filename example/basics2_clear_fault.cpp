/**
 * @example basics2_clear_fault.cpp
 * This tutorial clears minor or critical faults, if any, of the connected robots.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/drdk/robot_pair.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>

#include <iostream>
#include <string>
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
        ">>> Tutorial description <<<\nThis tutorial clears minor or critical faults, if any, of "
        "the connected robots.\n");

    try {
        // RDK Initialization
        // =========================================================================================
        // Instantiate robot pair interface
        drdk::RobotPair robot_pair({left_robot_sn, right_robot_sn});

        // Fault Clearing
        // =========================================================================================
        // Clear fault on the connected robots if any
        if (robot_pair.fault()) {
            spdlog::warn("Fault occurred on one of the connected robots, trying to clear ...");
            // Try to clear the fault
            auto result = robot_pair.ClearFault();
            if (!(result.first && result.second)) {
                spdlog::error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            spdlog::info("Fault on the connected robot is cleared");
        } else {
            spdlog::info("No fault on the connected robot");
        }
    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
