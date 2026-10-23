/**
 * @example basics5_plan_execution.cpp
 * This tutorial executes plans selected by the user from the lists of available plans of both
 * robots. A plan is a pre-written script to execute a series of robot primitives with pre-defined
 * transition conditions between 2 adjacent primitives. Users can use Flexiv Elements to compose
 * their own plan and assign to the robot, which will appear in the plan list.
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

/** @brief Print plan info of both robots @ 1Hz while any of them is busy */
void PrintPlanInfo(drdk::RobotPair& robot_pair)
{
    while (robot_pair.busy()) {
        auto plan_info = robot_pair.plan_info();
        std::cout << "Current plan info of the left robot:" << std::endl;
        std::cout << plan_info.first << std::endl;
        std::cout << "Current plan info of the right robot:" << std::endl;
        std::cout << plan_info.second << std::endl;
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
    std::cout << ">>> Tutorial description <<<\nThis tutorial executes plans selected by the user "
                 "from the lists of available plans of both robots. A plan is a pre-written script "
                 "to execute a series of robot primitives with pre-defined transition conditions "
                 "between 2 adjacent primitives. Users can use Flexiv Elements to compose their "
                 "own plan and assign to the robot, which will appear in the plan list.\n"
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

        // Execute Plans
        // =========================================================================================
        // Switch both robots to plan execution mode
        robot_pair.SwitchMode(rdk::Mode::NRT_PLAN_EXECUTION);

        while (true) {
            // Monitor fault on the connected robots
            if (robot_pair.fault()) {
                throw std::runtime_error("Fault occurred on the connected robots, exiting ...");
            }

            // Get user input
            std::cout << "Choose an action:" << std::endl;
            std::cout << "[1] Show available plans" << std::endl;
            std::cout << "[2] Execute plans by index" << std::endl;
            std::cout << "[3] Execute plans by name" << std::endl;

            std::string input_buffer;
            std::cin >> input_buffer;
            int user_input = std::stoi(input_buffer);

            switch (user_input) {
                // Get and show plan lists
                case 1: {
                    auto plan_lists = robot_pair.plan_list();
                    std::cout << "Available plans of the left robot:" << std::endl;
                    for (size_t i = 0; i < plan_lists.first.size(); i++) {
                        std::cout << "[" << i << "] " << plan_lists.first[i] << std::endl;
                    }
                    std::cout << "Available plans of the right robot:" << std::endl;
                    for (size_t i = 0; i < plan_lists.second.size(); i++) {
                        std::cout << "[" << i << "] " << plan_lists.second[i] << std::endl;
                    }
                    std::cout << std::endl;
                } break;
                // Execute plans by index
                case 2: {
                    unsigned int left_index = 0;
                    unsigned int right_index = 0;
                    std::cout << "Enter plan index to execute on the left robot:" << std::endl;
                    std::cin >> left_index;
                    std::cout << "Enter plan index to execute on the right robot:" << std::endl;
                    std::cin >> right_index;
                    // Allow the plans to continue their execution even if the DRDK program is
                    // closed or the connection is lost
                    robot_pair.ExecutePlan({left_index, right_index}, true);

                    // Print plan info while the current plans are running
                    PrintPlanInfo(robot_pair);
                } break;
                // Execute plans by name
                case 3: {
                    std::string left_name;
                    std::string right_name;
                    std::cout << "Enter plan name to execute on the left robot:" << std::endl;
                    std::cin >> left_name;
                    std::cout << "Enter plan name to execute on the right robot:" << std::endl;
                    std::cin >> right_name;
                    // Allow the plans to continue their execution even if the DRDK program is
                    // closed or the connection is lost
                    robot_pair.ExecutePlan({left_name, right_name}, true);

                    // Print plan info while the current plans are running
                    PrintPlanInfo(robot_pair);
                } break;
                default:
                    std::cerr << "[warn] Invalid input" << std::endl;
                    break;
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
