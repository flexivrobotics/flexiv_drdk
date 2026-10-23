/**
 * @example intermediate6_realtime_cartesian_motion_force_control.cpp
 * This tutorial runs real-time Cartesian-space unified motion-force control on both robots. The Z
 * axis of the chosen reference frame will be activated for explicit force control, while the rest
 * axes in the same reference frame will stay motion controlled.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/drdk/robot_pair.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>

#include <iostream>
#include <string>
#include <cmath>
#include <limits>
#include <thread>
#include <atomic>

using namespace flexiv;

namespace {
/** RT loop period [sec] */
constexpr double kLoopPeriod = 0.001;

/** TCP sine-sweep amplitude [m] */
constexpr double kSwingAmp = 0.1;

/** TCP sine-sweep frequency [Hz] */
constexpr double kSwingFreq = 0.3;

/** Pressing force to apply during the unified motion-force control [N] */
constexpr double kPressingForce = 5.0;

/** Cartesian linear velocity used to search for contact [m/s] */
constexpr double kSearchVelocity = 0.02;

/** Maximum distance to travel when searching for contact [m] */
constexpr double kSearchDistance = 1.0;

/** Maximum contact wrench during contact search for soft contact */
const std::array<double, rdk::kCartDoF> kMaxWrenchForContactSearch
    = {10.0, 10.0, 10.0, 3.0, 3.0, 3.0};

/** Atomic signal to stop scheduler tasks */
std::atomic<bool> g_stop_sched = {false};
}

/** @brief Print program usage help */
void PrintHelp()
{
    // clang-format off
    std::cout << "Required arguments: [left_robot_sn] [right_robot_sn]" << std::endl;
    std::cout << "    left_robot_sn: Serial number of the left robot to connect. Remove any space, e.g. Rizon4s-100001" << std::endl;
    std::cout << "    right_robot_sn: Serial number of the right robot to connect. Remove any space, e.g. Rizon4s-100002" << std::endl;
    std::cout << "Optional arguments: [--TCP] [--polish]" << std::endl;
    std::cout << "    --TCP: use TCP frame as reference frame for force control, otherwise use world frame" << std::endl;
    std::cout << "    --polish: run a simple polish motion along XY plane in world frame, otherwise hold robot motion in non-force-control axes"
              << std::endl
              << std::endl;
    // clang-format on
}

/** @brief Norm of the sensed external force applied on a robot's TCP */
double ExtForceNorm(const rdk::RobotStates& states)
{
    Eigen::Vector3d ext_force = {states.ext_wrench_in_world[0], states.ext_wrench_in_world[1],
        states.ext_wrench_in_world[2]};
    return ext_force.norm();
}

/** Callback function for realtime periodic task */
void PeriodicTask(drdk::RobotPair& robot_pair,
    const std::pair<std::array<double, rdk::kPoseSize>, std::array<double, rdk::kPoseSize>>&
        init_pose,
    rdk::CoordType force_ctrl_frame, bool enable_polish)
{
    // Local periodic loop counter
    static uint64_t loop_counter = 0;

    try {
        // Monitor fault on the connected robots
        if (robot_pair.fault()) {
            throw std::runtime_error(
                "PeriodicTask: Fault occurred on the connected robots, exiting ...");
        }

        // Initialize target poses to initial poses
        auto target_pose = init_pose;

        // Set Fz according to reference frame to achieve a "pressing down" behavior
        double Fz = 0.0;
        if (force_ctrl_frame == rdk::CoordType::WORLD) {
            Fz = kPressingForce;
        } else if (force_ctrl_frame == rdk::CoordType::TCP) {
            Fz = -kPressingForce;
        }
        std::array<double, rdk::kCartDoF> target_wrench = {0.0, 0.0, Fz, 0.0, 0.0, 0.0};

        // Apply constant force along Z axis of chosen reference frame, and do a simple polish
        // motion along XY plane in the shared world frame
        if (enable_polish) {
            // Create motion commands to sine-sweep along Y direction
            double offset = kSwingAmp * sin(2 * M_PI * kSwingFreq * loop_counter * kLoopPeriod);
            target_pose.first[1] = init_pose.first[1] + offset;
            target_pose.second[1] = init_pose.second[1] + offset;
            // Command target poses and target wrenches
            robot_pair.StreamCartesianMotionForce(target_pose, {target_wrench, target_wrench});
        }
        // Apply constant force along Z axis of chosen reference frame, and hold motions in all
        // other axes
        else {
            // Command initial poses and target wrenches
            robot_pair.StreamCartesianMotionForce(init_pose, {target_wrench, target_wrench});
        }

        // Increment loop counter
        loop_counter++;

    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        g_stop_sched = true;
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
    std::cout << ">>> Tutorial description <<<\nThis tutorial runs real-time Cartesian-space "
                 "unified motion-force control on both robots. The Z axis of the chosen reference "
                 "frame will be activated for explicit force control, while the rest axes in the "
                 "same reference frame will stay motion controlled.\n"
              << std::endl;

    // The reference frame used for force control, see RobotPair::SetForceControlFrame()
    auto force_ctrl_frame = rdk::CoordType::WORLD;
    if (rdk::utility::ProgramArgsExist(argc, argv, "--TCP")) {
        std::cout << "Reference frame used for force control: robot TCP frame" << std::endl;
        force_ctrl_frame = rdk::CoordType::TCP;
    } else {
        std::cout << "Reference frame used for force control: robot world frame" << std::endl;
    }

    // Whether to enable polish motion
    bool enable_polish = false;
    if (rdk::utility::ProgramArgsExist(argc, argv, "--polish")) {
        std::cout << "Robots will run a polish motion along XY plane in the shared world frame"
                  << std::endl;
        enable_polish = true;
    } else {
        std::cout << "Robots will hold their motion in all non-force-controlled axes" << std::endl;
    }

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

        // Zero Force-torque Sensors
        // =========================================================================================
        robot_pair.SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        // IMPORTANT: must zero force/torque sensor offset for accurate force/torque measurement
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

        // Search for Contact
        // =========================================================================================
        // NOTE: there are several ways to do contact search, such as using primitives, or real-time
        // and non-real-time direct motion controls, etc. Here we use non-real-time direct Cartesian
        // control for example.
        std::cout << "Searching for contact ..." << std::endl;

        // Set initial poses to current TCP poses in the shared world frame
        auto states = robot_pair.states();
        auto init_pose = std::make_pair(states.first.tcp_pose, states.second.tcp_pose);
        std::cout << "Initial TCP pose of the left robot set to [position 3x1, rotation "
                     "(quaternion) 4x1]: "
                  << rdk::utility::Arr2Str(init_pose.first) << std::endl;
        std::cout << "Initial TCP pose of the right robot set to [position 3x1, rotation "
                     "(quaternion) 4x1]: "
                  << rdk::utility::Arr2Str(init_pose.second) << std::endl;

        // Use non-real-time mode to make the robots go to a set point with their own motion
        // generators
        robot_pair.SwitchMode(rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);

        // Search for contact with max contact wrench set to a small value for making soft contact
        robot_pair.SetMaxContactWrench({kMaxWrenchForContactSearch, kMaxWrenchForContactSearch});

        // Set target points along -Z direction and expect contact to happen during the travel
        auto target_pose = init_pose;
        target_pose.first[2] -= kSearchDistance;
        target_pose.second[2] -= kSearchDistance;

        // Send target points to both robots to start searching for contact and limit the velocity.
        // Keep target wrench 0 at this stage since we are not doing force control yet
        robot_pair.SendCartesianMotionForce(
            target_pose, {}, {}, {kSearchVelocity, kSearchVelocity});

        // Use a while loop to poll robot states and check if a contact is made on both robots. A
        // robot that has made contact keeps pressing softly, limited by the max contact wrench,
        // while waiting for the other robot
        std::pair<bool, bool> is_contacted = {false, false};
        while (!is_contacted.first || !is_contacted.second) {
            states = robot_pair.states();

            // Contact is considered to be made if sensed TCP force exceeds the threshold
            if (!is_contacted.first && ExtForceNorm(states.first) > kPressingForce) {
                is_contacted.first = true;
                std::cout << "Contact detected at left robot TCP" << std::endl;
            }
            if (!is_contacted.second && ExtForceNorm(states.second) > kPressingForce) {
                is_contacted.second = true;
                std::cout << "Contact detected at right robot TCP" << std::endl;
            }
            // Check at 1ms interval
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Configure Force Control
        // =========================================================================================
        // Uncomment the following lines to enable passive force control, otherwise active force
        // control is used by default. Passive force control can only be configured in IDLE mode.
        // See function doc for more details
        /* robot_pair.SwitchMode(rdk::Mode::IDLE);
        robot_pair.SetPassiveForceControl({true, true}); */

        // Switch both robots to real-time mode for continuous motion force control
        robot_pair.SwitchMode(rdk::Mode::RT_CARTESIAN_MOTION_FORCE);

        // Set force control reference frame based on program argument. See function doc for more
        // details
        robot_pair.SetForceControlFrame({force_ctrl_frame, force_ctrl_frame});

        // Set which Cartesian axis(s) to activate for force control. See function doc for more
        // details. Here we only active Z axis
        std::array<bool, rdk::kCartDoF> force_ctrl_axes = {false, false, true, false, false, false};
        robot_pair.SetForceControlAxis({force_ctrl_axes, force_ctrl_axes});

        // NOTE: motion control always uses the shared world frame, while force control can use
        // either world or TCP frame as reference frame

        // Start Unified Motion Force Control
        // =========================================================================================
        // Disable max contact wrench regulation. Need to do this AFTER the force control in Z axis
        // is activated (i.e. motion control disabled in Z axis) and the motion force control mode
        // is entered, this way the contact force along Z axis is explicitly regulated and will not
        // spike after the max contact wrench regulation for motion control is disabled
        std::array<double, rdk::kCartDoF> inf;
        inf.fill(std::numeric_limits<double>::infinity());
        robot_pair.SetMaxContactWrench({inf, inf});

        // Update initial poses to current TCP poses
        states = robot_pair.states();
        init_pose = std::make_pair(states.first.tcp_pose, states.second.tcp_pose);

        // Create real-time scheduler to run periodic tasks
        rdk::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.AddTask(std::bind(PeriodicTask, std::ref(robot_pair), std::ref(init_pose),
                              force_ctrl_frame, enable_polish),
            "HP periodic", 1, scheduler.max_priority());
        // Start all added tasks
        scheduler.Start();

        // Block and wait for signal to stop scheduler tasks
        while (!g_stop_sched) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        // Received signal to stop scheduler tasks
        scheduler.Stop();

    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
