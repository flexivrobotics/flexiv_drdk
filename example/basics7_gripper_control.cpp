/**
 * @example basics7_gripper_control.cpp
 * This tutorial simultaneously does position and force (if available) control of a pair of
 * grippers supported by Flexiv, which are installed respectively on the pair of robots.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/drdk/robot_pair.hpp>
#include <flexiv/drdk/gripper_pair.hpp>
#include <flexiv/rdk/tool.hpp>
#include <flexiv/rdk/utility.hpp>

#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <limits>
#include <thread>
#include <atomic>

using namespace flexiv;

namespace {
/** Global flag: whether the gripper control tasks are finished */
std::atomic<bool> g_finished = {false};
}

/** @brief Print program usage help */
void PrintHelp()
{
    // clang-format off
    std::cout << "Required arguments: [left_robot_sn] [right_robot_sn] [left_gripper_name] [right_gripper_name]" << std::endl;
    std::cout << "    left_robot_sn: Serial number of the left robot to connect. Remove any space, e.g. Rizon4s-100001" << std::endl;
    std::cout << "    right_robot_sn: Serial number of the right robot to connect. Remove any space, e.g. Rizon4s-100002" << std::endl;
    std::cout << "    left_gripper_name: Full name of the gripper installed on the left robot, can be found in Flexiv Elements -> Settings -> Device" << std::endl;
    std::cout << "    right_gripper_name: Full name of the gripper installed on the right robot, can be found in Flexiv Elements -> Settings -> Device" << std::endl;
    std::cout << "Optional arguments: None" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

/** @brief Print parameters of a gripper */
void PrintGripperParams(const rdk::GripperParams& params)
{
    std::cout << std::fixed << std::setprecision(3) << "{\n"
              << "name: " << params.name << "\nmin_width: " << params.min_width
              << "\nmax_width: " << params.max_width << "\nmin_force: " << params.min_force
              << "\nmax_force: " << params.max_force << "\nmin_vel: " << params.min_vel
              << "\nmax_vel: " << params.max_vel << "\n}" << std::endl;
}

/** @brief Print states data of both grippers @ 1Hz */
void PrintGripperStates(drdk::GripperPair& gripper_pair)
{
    while (!g_finished) {
        // Print all gripper states in JSON format using the built-in ostream operator overloading
        auto states = gripper_pair.states();
        std::cout << "Current states of the left gripper:" << std::endl;
        std::cout << states.first << std::endl;
        std::cout << "Current states of the right gripper:" << std::endl;
        std::cout << states.second << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main(int argc, char* argv[])
{
    // Program Setup
    // =============================================================================================
    // Parse parameters
    if (argc < 5 || rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }

    // Serial numbers of the left and right robot respectively
    std::string left_robot_sn = argv[1];
    std::string right_robot_sn = argv[2];

    // Names of the grippers installed on the left and right robot respectively
    std::string left_gripper_name = argv[3];
    std::string right_gripper_name = argv[4];

    // Print description
    std::cout << ">>> Tutorial description <<<\nThis tutorial simultaneously does position and "
                 "force (if available) control of a pair of grippers supported by Flexiv, which "
                 "are installed respectively on the pair of robots.\n"
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

        // Gripper Control
        // =========================================================================================
        // Instantiate gripper pair control interface
        drdk::GripperPair gripper_pair(robot_pair);

        // Instantiate tool interfaces. Gripper is categorized as both a device and a tool. The
        // device attribute allows a gripper to be interactively controlled by the user; whereas the
        // tool attribute tells the robot to account for its mass properties and TCP location. DRDK
        // does not provide a tool interface for the robot pair, so an rdk::Tool interface is
        // instantiated for each of the underlying rdk::Robot instances
        rdk::Tool left_tool(*robot_pair.instances().first);
        rdk::Tool right_tool(*robot_pair.instances().second);

        // Enable the specified grippers as devices. This is equivalent to enabling the specified
        // grippers in Flexiv Elements -> Settings -> Device
        std::cout << "Enabling grippers [" << left_gripper_name << "] and [" << right_gripper_name
                  << "]" << std::endl;
        gripper_pair.Enable({left_gripper_name, right_gripper_name});

        // Print parameters of the enabled grippers
        auto params = gripper_pair.params();
        std::cout << "Left gripper params:" << std::endl;
        PrintGripperParams(params.first);
        std::cout << "Right gripper params:" << std::endl;
        PrintGripperParams(params.second);

        // Switch robot tools to grippers so the gravity compensation and TCP location is updated
        std::cout << "Switching tools of the left and right robot to [" << left_gripper_name
                  << "] and [" << right_gripper_name << "]" << std::endl;
        left_tool.Switch(left_gripper_name);
        right_tool.Switch(right_gripper_name);

        // User needs to determine if the grippers require manual initialization
        int choice = 0;
        std::cout << "Manually trigger initialization for the grippers now? Choose Yes if they are "
                     "48v Grav grippers"
                  << std::endl;
        std::cout << "[1] No, they have already initialized automatically when power on"
                  << std::endl;
        std::cout << "[2] Yes, they do not initialize themselves when power on" << std::endl;
        std::cin >> choice;

        // Trigger manual initialization based on choice
        if (choice == 1) {
            std::cout << "Skipped manual initialization" << std::endl;
        } else if (choice == 2) {
            gripper_pair.Init();
            // User determines if the manual initialization is finished
            std::cout << "Triggered manual initialization, press Enter when the initialization is "
                         "finished to continue"
                      << std::endl;
            std::cin.get();
            std::cin.get();
        } else {
            std::cerr << "[error] Invalid choice" << std::endl;
            return 1;
        }

        // Start a separate thread to print gripper states
        std::thread print_thread(PrintGripperStates, std::ref(gripper_pair));

        // Position control
        std::cout << "Closing grippers" << std::endl;
        gripper_pair.Move({0.01, 0.01}, {0.1, 0.1}, {20, 20});
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "Opening grippers" << std::endl;
        gripper_pair.Move({0.09, 0.09}, {0.1, 0.1}, {20, 20});
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Stop
        std::cout << "Closing grippers" << std::endl;
        gripper_pair.Move({0.01, 0.01}, {0.1, 0.1}, {20, 20});
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Stopping grippers" << std::endl;
        gripper_pair.Stop();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "Closing grippers" << std::endl;
        gripper_pair.Move({0.01, 0.01}, {0.1, 0.1}, {20, 20});
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "Opening grippers" << std::endl;
        gripper_pair.Move({0.09, 0.09}, {0.1, 0.1}, {20, 20});
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Stopping grippers" << std::endl;
        gripper_pair.Stop();
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Force control, if available on both grippers (sensed force is not zero)
        auto states = gripper_pair.states();
        if (fabs(states.first.force) > std::numeric_limits<double>::epsilon()
            && fabs(states.second.force) > std::numeric_limits<double>::epsilon()) {
            std::cout << "Grippers running zero force control" << std::endl;
            gripper_pair.Grasp({0, 0});
            // Exit after 10 seconds
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }

        // Finished, exit all threads
        gripper_pair.Stop();
        g_finished = true;
        std::cout << "Program finished" << std::endl;
        print_thread.join();

    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
