/**
 * @example basics8_auto_recovery.cpp
 * This tutorial runs an automatic recovery process if the safety system of either robot is in
 * recovery state. See drdk::RobotPair::recovery() and RDK manual for more details.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/drdk/robot_pair.hpp>
#include <flexiv/rdk/utility.hpp>

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
    std::cout << ">>> Tutorial description <<<\nThis tutorial runs an automatic recovery process "
                 "if the safety system of either robot is in recovery state. See "
                 "drdk::RobotPair::recovery() and RDK manual for more details.\n"
              << std::endl;

    try {
        // DRDK Initialization
        // =========================================================================================
        // Instantiate robot pair interface
        drdk::RobotPair robot_pair({left_robot_sn, right_robot_sn});

        // Enable the pair of robots, make sure the E-stop is released before enabling
        std::cout << "Enabling robots ..." << std::endl;
        robot_pair.Enable();

        // Run Auto-recovery
        // =========================================================================================
        // If the system is in recovery state, we can't use operational() to tell if the enabling
        // process is done, so just wait long enough for the process to finish
        std::this_thread::sleep_for(std::chrono::seconds(8));

        // Run automatic recovery if either robot is in recovery state, the involved joints will
        // start to move back into allowed position range
        auto recovery = robot_pair.recovery();
        if (recovery.first || recovery.second) {
            std::cout << "Left robot in recovery state: " << recovery.first << std::endl;
            std::cout << "Right robot in recovery state: " << recovery.second << std::endl;
            robot_pair.RunAutoRecovery();
        }
        // Otherwise both systems are normal, do nothing
        else {
            std::cout << "Neither robot is in recovery state, nothing to be done, exiting ..."
                      << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
