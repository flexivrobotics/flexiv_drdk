/**
 * @example basics4_primitive_execution.cpp
 * This tutorial executes several basic robot primitives (unit skills) simultaneously on both robots
 * in the pair. For detailed documentation on all available primitives, please see [Flexiv
 * Primitives](https://www.flexiv.com/primitives/).
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
    std::cout << ">>> Tutorial description <<<\nThis tutorial executes several basic robot "
                 "primitives (unit skills) simultaneously on both robots in the pair. For detailed "
                 "documentation on all available primitives, please see [Flexiv "
                 "Primitives](https://www.flexiv.com/primitives/).\n"
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

        // Execute Primitives
        // =========================================================================================
        // Switch both robots to primitive execution mode
        robot_pair.SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);

        // (1) Go to home pose
        // -----------------------------------------------------------------------------------------
        // All parameters of the "Home" primitive are optional, thus we can skip the parameters and
        // the default values will be used
        std::cout << "Executing primitive: Home" << std::endl;

        // Send command to both robots
        robot_pair.ExecutePrimitive({"Home", "Home"}, {});

        // Wait for both robots to reach target
        while (!std::get<int>(robot_pair.primitive_states().first["reachedTarget"])
               || !std::get<int>(robot_pair.primitive_states().second["reachedTarget"])) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // (2) Move robot joints to target positions
        // -----------------------------------------------------------------------------------------
        // Required parameters:
        //     target: final joint positions, unit: degrees
        //         {arm joint positions}, {external axis joint positions}
        // Optional parameters:
        //     waypoints: waypoints to pass before reaching the target
        //         (same format as above, but can be more than one)
        //     vel: TCP linear velocity, unit: m/s
        std::cout << "Executing primitive: MoveJ" << std::endl;

        // The input parameters are given respectively for the left and right robot. Here both
        // robots use the same parameters, but they can be different
        std::map<std::string, rdk::FlexivDataTypes> move_j_params = {
            {"target", rdk::JPos({30, -45, 0, 90, 0, 40, 30}, {-50, 30})},
            {"waypoints",
                std::vector<rdk::JPos> {rdk::JPos({10, -30, 10, 30, 10, 15, 10}, {-15, 10}),
                    rdk::JPos({20, -60, -10, 60, -10, 30, 20}, {-30, 20})}},
        };

        // Send command to both robots
        robot_pair.ExecutePrimitive({"MoveJ", "MoveJ"}, {move_j_params, move_j_params});

        // Most primitives won't exit by themselves and require users to explicitly trigger
        // transitions based on specific primitive states. Here we check if the primitive state
        // [reachedTarget] becomes true on both robots and trigger the transition manually by
        // sending a new primitive command.
        while (!std::get<int>(robot_pair.primitive_states().first["reachedTarget"])
               || !std::get<int>(robot_pair.primitive_states().second["reachedTarget"])) {
            // Print current primitive states of both robots
            auto pt_states = robot_pair.primitive_states();
            std::cout << "Current primitive states of the left robot:" << std::endl;
            for (const auto& st : pt_states.first) {
                std::cout << st.first << ": " << rdk::utility::FlexivTypes2Str(st.second);
                std::cout << std::endl;
            }
            std::cout << "Current primitive states of the right robot:" << std::endl;
            for (const auto& st : pt_states.second) {
                std::cout << st.first << ": " << rdk::utility::FlexivTypes2Str(st.second);
                std::cout << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // (3) Move robot TCP to a target pose in world (base) frame
        // -----------------------------------------------------------------------------------------
        // Required parameters:
        //     target: final TCP pose, unit: m and degrees
        //         {pos_x, pos_y, pos_z}, {rot_x, rot_y, rot_z}, {ref_frame, ref_point}
        // Optional parameters:
        //     waypoints: waypoints to pass before reaching the target
        //         (same format as above, but can be more than one)
        //     vel: TCP linear velocity, unit: m/s
        // NOTE: The rotations use Euler ZYX convention, rot_x means Euler ZYX angle around X axis
        // NOTE: The WORLD frame used by primitives is each robot's own world frame, not the shared
        // world frame of the robot pair
        std::cout << "Executing primitive: MoveL" << std::endl;

        // Input parameters of the primitive, used by both robots
        std::map<std::string, rdk::FlexivDataTypes> move_l_params = {
            {"target", rdk::Coord({0.65, -0.3, 0.2}, {180, 0, 180}, {"WORLD", "WORLD_ORIGIN"})},
            {"waypoints",
                std::vector<rdk::Coord> {
                    rdk::Coord({0.45, 0.1, 0.2}, {180, 0, 180}, {"WORLD", "WORLD_ORIGIN"}),
                    rdk::Coord({0.45, -0.3, 0.2}, {180, 0, 180}, {"WORLD", "WORLD_ORIGIN"})}},
            {"vel", 0.6},
            {"zoneRadius", "Z50"},
        };

        // Send command to both robots
        robot_pair.ExecutePrimitive({"MoveL", "MoveL"}, {move_l_params, move_l_params});

        // Wait for both robots to reach target
        while (!std::get<int>(robot_pair.primitive_states().first["reachedTarget"])
               || !std::get<int>(robot_pair.primitive_states().second["reachedTarget"])) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // (4) Another MoveL that uses TCP frame
        // -----------------------------------------------------------------------------------------
        // In this example the reference frame is changed from WORLD::WORLD_ORIGIN to TRAJ::START,
        // which represents the current TCP frame
        std::cout << "Executing primitive: MoveL" << std::endl;

        // Example to convert target quaternion [w,x,y,z] to Euler ZYX using utility functions
        std::array<double, 4> target_quat = {0.9185587, 0.1767767, 0.3061862, 0.1767767};
        // ZYX = [30, 30, 30] degrees
        auto target_euler_deg = rdk::utility::Rad2Deg(rdk::utility::Quat2EulerZYX(target_quat));

        // Input parameters of the primitive, used by both robots. This motion will hold current TCP
        // position and only do rotation
        std::map<std::string, rdk::FlexivDataTypes> rotate_params = {
            {"target", rdk::Coord({0.0, 0.0, 0.0}, target_euler_deg, {"TRAJ", "START"})},
            {"vel", 0.2},
        };

        // Send command to both robots
        robot_pair.ExecutePrimitive({"MoveL", "MoveL"}, {rotate_params, rotate_params});

        // Wait for both robots to reach target
        while (!std::get<int>(robot_pair.primitive_states().first["reachedTarget"])
               || !std::get<int>(robot_pair.primitive_states().second["reachedTarget"])) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // All done, stop both robots and put into IDLE mode
        robot_pair.Stop();

    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
