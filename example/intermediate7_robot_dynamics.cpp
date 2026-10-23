/**
 * @example intermediate7_robot_dynamics.cpp
 * This tutorial runs the integrated dynamics engine of both robots to obtain robot Jacobian, mass
 * matrix, and gravity torques. Also checks reachability of a Cartesian pose.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/drdk/robot_pair.hpp>
#include <flexiv/rdk/model.hpp>
#include <flexiv/rdk/utility.hpp>

#include <iostream>
#include <iomanip>
#include <string>
#include <thread>
#include <chrono>

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

/** @brief Update a robot's dynamics engine with the given robot states, then compute and print the
 * gravity vector, mass matrix, and Jacobian */
void ComputeDynamics(rdk::Model& model, const rdk::RobotStates& states)
{
    // Mark timer start point
    auto tic = std::chrono::high_resolution_clock::now();

    // Update robot model in dynamics engine
    model.Update(states.q, states.dtheta);

    // Compute gravity vector
    auto g = model.g();

    // Compute mass matrix
    auto M = model.M();

    // Compute Jacobian
    auto J = model.J("flange");

    // Mark timer end point and get loop time
    auto toc = std::chrono::high_resolution_clock::now();
    auto computation_time
        = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count();

    // Print time used to compute g, M, J
    std::cout << "Computation time = " << computation_time << " us" << std::endl;
    // Print gravity
    std::cout << "g = \n" << std::fixed << std::setprecision(5) << g.transpose() << std::endl;
    // Print mass matrix
    std::cout << "M = \n" << std::fixed << std::setprecision(5) << M << std::endl;
    // Print Jacobian
    std::cout << "J = \n" << std::fixed << std::setprecision(5) << J << std::endl;
    std::cout << std::endl;
}

/** @brief Check reachability of a Cartesian pose based on a robot's current pose */
void CheckReachability(const rdk::Model& model, const rdk::RobotStates& states)
{
    auto pose_to_check = states.tcp_pose;
    pose_to_check[0] += 0.1;
    std::cout << "Checking reachability of Cartesian pose [" << rdk::utility::Arr2Str(pose_to_check)
              << "]" << std::endl;
    auto result = model.reachable(pose_to_check, states.q, true);
    std::cout << "Got a result: reachable = " << result.first << ", IK solution = ["
              << rdk::utility::Vec2Str(result.second) << "]" << std::endl;
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
    std::cout << ">>> Tutorial description <<<\nThis tutorial runs the integrated dynamics engine "
                 "of both robots to obtain robot Jacobian, mass matrix, and gravity torques. Also "
                 "checks reachability of a Cartesian pose.\n"
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

        // Robot Dynamics
        // =========================================================================================
        // Initialize dynamics engines. DRDK does not provide a dynamics engine for the robot pair,
        // so an rdk::Model is instantiated for each of the underlying rdk::Robot instances
        rdk::Model left_model(*robot_pair.instances().first);
        rdk::Model right_model(*robot_pair.instances().second);

        // Step dynamics engines 5 times
        for (size_t i = 0; i < 5; i++) {
            auto states = robot_pair.states();
            std::cout << "Left robot:" << std::endl;
            ComputeDynamics(left_model, states.first);
            std::cout << "Right robot:" << std::endl;
            ComputeDynamics(right_model, states.second);
        }

        // Check reachability of a Cartesian pose based on the current pose of each robot. The
        // dynamics engine works in the robot's own world frame, so the states of the underlying
        // rdk::Robot instances are used here instead of RobotPair::states(), whose TCP poses are
        // expressed in the shared world frame
        std::cout << "Left robot:" << std::endl;
        CheckReachability(left_model, robot_pair.instances().first->states());
        std::cout << "Right robot:" << std::endl;
        CheckReachability(right_model, robot_pair.instances().second->states());

    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
