/**
 * @example intermediate8_bimanual_tensioned_motion.cpp
 * This tutorial uses the bimanual primitive MoveTensioned to squeeze an object between the
 * end-effectors of the two robots, then lift it up and put it back down while keeping the squeezing
 * force. The two robots are assumed to be mounted upright, side by side along the Y axis of the
 * shared world frame. Modify the base translations and holding poses below to match the actual
 * setup.
 * @warning An object must be held between the two end-effectors before the squeezing starts,
 * otherwise the two robots will keep moving towards each other.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/drdk/robot_pair.hpp>
#include <flexiv/drdk/bimanual_primitives.hpp>
#include <flexiv/rdk/utility.hpp>

#include <iostream>
#include <string>
#include <thread>

using namespace flexiv;

namespace {
/** Translations of the left and right robot's base frame in the shared world frame [m] */
const std::array<double, 3> kLeftTransInWorld = {0.0, 0.3, 0.0};
const std::array<double, 3> kRightTransInWorld = {0.0, -0.3, 0.0};

/** TCP poses of the left and right robot in the shared world frame to hold an object between them,
 * with the two end-effectors facing each other 20 cm apart [m][] */
const std::array<double, rdk::kPoseSize> kLeftHoldPose
    = {0.5, 0.1, 0.3, 0.7071068, 0.7071068, 0.0, 0.0};
const std::array<double, rdk::kPoseSize> kRightHoldPose
    = {0.5, -0.1, 0.3, 0.7071068, -0.7071068, 0.0, 0.0};

/** Squeezing force applied on the object [N] */
constexpr double kSqueezeForce = 10.0;

/** Height to lift the object [m] */
constexpr double kLiftHeight = 0.1;

/** Linear and angular velocity of the bimanual motion [m/s] [rad/s] */
constexpr double kLinearVel = 0.05;
constexpr double kAngularVel = 0.2;
}

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

/** @brief Move both robots to the holding poses and wait for them to stop */
void MoveToHoldPoses(drdk::RobotPair& robot_pair)
{
    robot_pair.SendCartesianMotionForce({kLeftHoldPose, kRightHoldPose});
    do {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    } while (!robot_pair.stopped());
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
    std::cout << ">>> Tutorial description <<<\nThis tutorial uses the bimanual primitive "
                 "MoveTensioned to squeeze an object between the end-effectors of the two robots, "
                 "then lift it up and put it back down while keeping the squeezing force.\n"
              << std::endl;

    try {
        // DRDK Initialization
        // =========================================================================================
        // Instantiate robot pair interface. The base translations are needed by the bimanual
        // primitives to know where the two robots are relative to each other
        drdk::RobotPair robot_pair(
            {left_robot_sn, right_robot_sn}, {kLeftTransInWorld, kRightTransInWorld});

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

        // Zero Force-torque Sensors
        // =========================================================================================
        robot_pair.SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        // IMPORTANT: must zero force/torque sensor offset, because MoveTensioned measures the
        // squeezing force using the force/torque sensors
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

        // Hold Object
        // =========================================================================================
        // Move the two end-effectors to face each other
        std::cout << "Moving to holding poses" << std::endl;
        robot_pair.SwitchMode(rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        MoveToHoldPoses(robot_pair);

        // Wait for the user to place an object between the two end-effectors
        std::cerr << "[warn] Place an object about 20 cm wide between the two end-effectors, then "
                     "press Enter to squeeze it. Without an object, the two robots will keep "
                     "moving towards each other"
                  << std::endl;
        std::cin.get();

        // Bimanual Tensioned Motion
        // =========================================================================================
        // Instantiate bimanual primitives interface
        drdk::BimanualPrimitives bimanual_primitives(robot_pair);

        // Squeeze the object, then lift it up. The left robot's TCP tracks the target pose, while
        // the right robot follows along and keeps squeezing the object. A negative tension force
        // means compression
        auto target_pose = robot_pair.states().first.tcp_pose;
        target_pose[2] += kLiftHeight;
        std::cout << "Squeezing the object with " << kSqueezeForce << " N and lifting it up by "
                  << kLiftHeight << " m" << std::endl;
        bimanual_primitives.MoveTensioned(
            drdk::ReferenceArm::LEFT, target_pose, -kSqueezeForce, kLinearVel, kAngularVel);

        // Put the object back down, this time using the right robot's TCP as reference
        target_pose = robot_pair.states().second.tcp_pose;
        target_pose[2] -= kLiftHeight;
        std::cout << "Putting the object back down" << std::endl;
        bimanual_primitives.MoveTensioned(
            drdk::ReferenceArm::RIGHT, target_pose, -kSqueezeForce, kLinearVel, kAngularVel);

        // Release Object
        // =========================================================================================
        // Wait for the user to hold the object before releasing it
        std::cerr << "[warn] Hold the object, then press Enter to release it" << std::endl;
        std::cin.get();

        // Move the two end-effectors back to the holding poses to release the squeezing force
        std::cout << "Releasing the object" << std::endl;
        MoveToHoldPoses(robot_pair);

        std::cout << "Program finished" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
