/**
 * @example basics3_self_collision_avoidance.cpp
 * This tutorial shows how to use the self collision monitor to keep the left and right robot from
 * hitting each other.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/drdk/robot_pair.hpp>
#include <flexiv/drdk/self_collision_monitor.hpp>
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
    std::cout << ">>> Tutorial description <<<\nThis tutorial shows how to use the self collision "
                 "monitor to keep the left and right robot from hitting each other.\n"
              << std::endl;

    try {
        // DRDK Initialization
        // =========================================================================================
        // Translation in world of the left and robot
        std::array<double, 3> left_trans_in_world = {0, 0.3, 0};
        std::array<double, 3> right_trans_in_world = {0, -0.3, 0};

        // Instantiate robot pair interface
        drdk::RobotPair robot_pair(
            {left_robot_sn, right_robot_sn}, {left_trans_in_world, right_trans_in_world});

        // Self Collision Monitor
        // =========================================================================================
        // Instantiate self collision monitor interface. The collision meshes of both robots are
        // downloaded from the connected robots. All links are checked for collision by default, the
        // second argument can be used to skip some of them, e.g. {"base_link"}
        drdk::SelfCollisionMonitor coll_monitor(robot_pair);

        // Use 10 cm safety distance, leaving some buffer for the robots to decelerate
        coll_monitor.SetMinDistance(0.1);
        std::cout << "Safety distance set to 10 cm" << std::endl;

        // Start collision monitor and keep it running in the background
        coll_monitor.Start();
        std::cout << "Self-collision monitor started" << std::endl;

        // Move the two robots to home pose
        robot_pair.SwitchMode(rdk::Mode::NRT_PLAN_EXECUTION);
        robot_pair.ExecutePlan({"PLAN-Home", "PLAN-Home"});
        do {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } while (robot_pair.busy());
        std::cout << "Robot pair reached home" << std::endl;

        // Move the two robots to the same Cartesian location in world
        std::cout << "Moving the robots towards each other" << std::endl;
        robot_pair.SwitchMode(rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        std::array<double, rdk::kPoseSize> target_pose = {0.5, 0.0, 0.3, 0, 0, 1, 0};
        robot_pair.SendCartesianMotionForce({target_pose, target_pose});

        // Both robots will stop before they hit each other
        do {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } while (!robot_pair.stopped());

        std::cout << "Program finished" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
