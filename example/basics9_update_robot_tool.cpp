/**
 * @example basics9_update_robot_tool.cpp
 * This tutorial shows how to online update and interact with the tools of both robots. All changes
 * made to the robot tool system will take effect immediately without needing to reboot. However,
 * the robots must be put into IDLE mode when making these changes.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/drdk/robot_pair.hpp>
#include <flexiv/rdk/tool.hpp>
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

/** @brief Print all configured tools currently in a robot's tools pool */
void PrintToolList(const rdk::Tool& tool)
{
    auto tool_list = tool.list();
    for (size_t i = 0; i < tool_list.size(); i++) {
        std::cout << "[" << i << "] " << tool_list[i] << std::endl;
    }
    std::cout << std::endl;
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
    std::cout << ">>> Tutorial description <<<\nThis tutorial shows how to online update and "
                 "interact with the tools of both robots. All changes made to the robot tool "
                 "system will take effect immediately without needing to reboot. However, the "
                 "robots must be put into IDLE mode when making these changes.\n"
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

        // Update Robot Tools
        // =========================================================================================
        // Make sure both robots are in IDLE mode
        robot_pair.SwitchMode(rdk::Mode::IDLE);

        // Instantiate tool interfaces. DRDK does not provide a tool interface for the robot pair,
        // so an rdk::Tool interface is instantiated for each of the underlying rdk::Robot instances
        rdk::Tool left_tool(*robot_pair.instances().first);
        rdk::Tool right_tool(*robot_pair.instances().second);

        // The same operations are done on both robots, so put their tool interfaces in a list
        const std::vector<std::pair<std::string, rdk::Tool*>> tools
            = {{"left", &left_tool}, {"right", &right_tool}};

        // Get and print a list of already configured tools currently in each robot's tools pool
        for (const auto& [robot_name, tool] : tools) {
            std::cout << "All configured tools of the " << robot_name << " robot:" << std::endl;
            PrintToolList(*tool);
        }

        // Get and print the current active tool of each robot
        for (const auto& [robot_name, tool] : tools) {
            std::cout << "Current active tool of the " << robot_name << " robot: " << tool->name()
                      << std::endl;
        }

        // Set name and parameters for a new tool
        std::string new_tool_name = "ExampleTool1";
        rdk::ToolParams new_tool_params;
        new_tool_params.mass = 0.9;
        new_tool_params.CoM = {0.0, 0.0, 0.057};
        new_tool_params.inertia = {2.768e-03, 3.149e-03, 5.64e-04, 0.0, 0.0, 0.0};
        new_tool_params.tcp_location = {0.0, -0.207, 0.09, 0.7071068, 0.7071068, 0.0, 0.0};

        // If there's already a tool with the same name in a robot's tools pool, then remove it
        // first, because duplicate tool names are not allowed
        for (const auto& [robot_name, tool] : tools) {
            if (tool->exist(new_tool_name)) {
                std::cerr << "[warn] Tool with the same name [" << new_tool_name
                          << "] already exists in the " << robot_name << " robot, removing it now"
                          << std::endl;
                // Switch to other tool or no tool (Flange) before removing the current tool
                tool->Switch("Flange");
                tool->Remove(new_tool_name);
            }
        }

        // Add the new tool to both robots
        for (const auto& [robot_name, tool] : tools) {
            std::cout << "Adding new tool [" << new_tool_name << "] to the " << robot_name
                      << " robot" << std::endl;
            tool->Add(new_tool_name, new_tool_params);
        }

        // Get and print the tools lists again, the new tool should appear at the end
        for (const auto& [robot_name, tool] : tools) {
            std::cout << "All configured tools of the " << robot_name << " robot:" << std::endl;
            PrintToolList(*tool);
        }

        // Switch both robots to the newly added tool, i.e. set it as the active tool
        for (const auto& [robot_name, tool] : tools) {
            std::cout << "Switching the " << robot_name << " robot to tool [" << new_tool_name
                      << "]" << std::endl;
            tool->Switch(new_tool_name);
        }

        // Get and print the current active tools again, should be the new tool
        for (const auto& [robot_name, tool] : tools) {
            std::cout << "Current active tool of the " << robot_name << " robot: " << tool->name()
                      << std::endl;
        }

        // Switch to other tool or no tool (Flange) before removing the current tool
        for (const auto& [robot_name, tool] : tools) {
            tool->Switch("Flange");
        }

        // Clean up by removing the new tool from both robots
        std::this_thread::sleep_for(std::chrono::seconds(2));
        for (const auto& [robot_name, tool] : tools) {
            std::cout << "Removing tool [" << new_tool_name << "] from the " << robot_name
                      << " robot" << std::endl;
            tool->Remove(new_tool_name);
        }

        std::cout << "Program finished" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
