/**
 * @example basics6_zero_force_torque_sensors.cpp
 * This tutorial zeros the force and torque sensors of both robots, which is a recommended (but not
 * mandatory) step before any operations that require accurate force/torque measurement.
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

/** @brief Print the current TCP force/moment readings of both robots */
void PrintTcpWrenches(const drdk::RobotPair& robot_pair)
{
    auto states = robot_pair.states();
    std::cout << "Left robot: " << rdk::utility::Arr2Str(states.first.ext_wrench_in_world)
              << "[N][Nm]" << std::endl;
    std::cout << "Right robot: " << rdk::utility::Arr2Str(states.second.ext_wrench_in_world)
              << "[N][Nm]" << std::endl;
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
    std::cout << ">>> Tutorial description <<<\nThis tutorial zeros the force and torque sensors "
                 "of both robots, which is a recommended (but not mandatory) step before any "
                 "operations that require accurate force/torque measurement.\n"
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

        // Zero Sensors
        // =========================================================================================
        // Get and print the current TCP force/moment readings
        std::cout << "TCP force and moment reading in world frame BEFORE sensor zeroing:"
                  << std::endl;
        PrintTcpWrenches(robot_pair);

        // Run the "ZeroFTSensor" primitive on both robots to automatically zero force and torque
        // sensors
        robot_pair.SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);
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

        // Get and print the current TCP force/moment readings
        std::cout << "TCP force and moment reading in world frame AFTER sensor zeroing:"
                  << std::endl;
        PrintTcpWrenches(robot_pair);

    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
