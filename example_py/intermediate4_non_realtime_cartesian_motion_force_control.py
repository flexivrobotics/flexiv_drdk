#!/usr/bin/env python

"""intermediate4_non_realtime_cartesian_motion_force_control.py

This tutorial runs non-real-time Cartesian-space unified motion-force control on both robots. The Z
axis of the chosen reference frame will be activated for explicit force control, while the rest
axes in the same reference frame will stay motion controlled.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import math
import argparse
import logging
import numpy as np  # pip install numpy
import flexivdrdk
from flexivrdk import Mode, CoordType  # for type definition

# Global constants
# ==================================================================================================
# TCP sine-sweep amplitude [m]
SWING_AMP = 0.1

# TCP sine-sweep frequency [Hz]
SWING_FREQ = 0.3

# Pressing force to apply during the unified motion-force control [N]
PRESSING_FORCE = 5.0

# Cartesian linear velocity used to search for contact [m/s]
SEARCH_VELOCITY = 0.02

# Maximum distance to travel when searching for contact [m]
SEARCH_DISTANCE = 1.0

# Maximum contact wrench during contact search for soft contact
MAX_WRENCH_FOR_CONTACT_SEARCH = [10.0, 10.0, 10.0, 3.0, 3.0, 3.0]


def main():
    # Program Setup
    # ==============================================================================================
    # Parse arguments
    argparser = argparse.ArgumentParser()
    # Required arguments
    argparser.add_argument(
        "left_robot_sn",
        help="Serial number of the left robot to connect. Remove any space, e.g. Rizon4s-100001",
    )
    argparser.add_argument(
        "right_robot_sn",
        help="Serial number of the right robot to connect. Remove any space, e.g. Rizon4s-100002",
    )
    argparser.add_argument(
        "frequency", help="Command frequency, 1 to 100 [Hz]", type=int
    )
    # Optional arguments
    argparser.add_argument(
        "--TCP",
        action="store_true",
        help="Use TCP frame as reference frame for force control, otherwise use world frame",
    )
    argparser.add_argument(
        "--polish",
        action="store_true",
        help="Run a simple polish motion along XY plane in world frame, otherwise hold robot motion in non-force-control axes",
    )
    args = argparser.parse_args()

    # Check if arguments are valid
    frequency = args.frequency
    assert frequency >= 1 and frequency <= 100, "Invalid <frequency> input"

    # Define alias
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    logger = logging.getLogger("Example")

    # Print description
    logger.info(
        ">>> Tutorial description <<<\nThis tutorial runs non-real-time Cartesian-space unified "
        "motion-force control on both robots. The Z axis of the chosen reference frame will be "
        "activated for explicit force control, while the rest axes in the same reference frame "
        "will stay motion controlled.\n"
    )

    # The reference frame to use, see RobotPair::SetForceControlFrame() for more details
    force_ctrl_frame = CoordType.WORLD
    if args.TCP:
        logger.info("Reference frame used for force control: robot TCP frame")
        force_ctrl_frame = CoordType.TCP
    else:
        logger.info("Reference frame used for force control: robot world frame")

    # Whether to enable polish motion
    if args.polish:
        logger.info(
            "Robots will run a polish motion along XY plane in the shared world frame"
        )
    else:
        logger.info("Robots will hold their motion in all non-force-controlled axes")

    try:
        # DRDK Initialization
        # ==========================================================================================
        # Instantiate robot pair interface
        robot_pair = flexivdrdk.RobotPair([args.left_robot_sn, args.right_robot_sn])

        # Clear fault on the connected robot if any
        if robot_pair.fault():
            logger.warning("Fault occurred on the connected robot, trying to clear ...")
            # Try to clear the fault on both robots
            result = robot_pair.ClearFault()
            # If fault is not cleared on both robots
            if not (result[0] and result[1]):
                logger.error("Fault cannot be cleared, exiting ...")
                return 1
            logger.info("Fault on the connected robot is cleared")

        # Enable the pair of robots, make sure the E-stop is released before enabling
        logger.info("Enabling robots ...")
        robot_pair.Enable()

        # Wait for the robot to become operational
        while not robot_pair.operational():
            time.sleep(1)

        logger.info("Both robots are now operational")

        # Move both robots to home pose
        logger.info("Moving to home pose")
        robot_pair.SwitchMode(Mode.NRT_PLAN_EXECUTION)
        robot_pair.ExecutePlan(["PLAN-Home", "PLAN-Home"])
        # Wait for the plans to finish
        while robot_pair.busy():
            time.sleep(1)

        # Zero Force-torque Sensors
        # =========================================================================================
        robot_pair.SwitchMode(Mode.NRT_PRIMITIVE_EXECUTION)
        # IMPORTANT: must zero force/torque sensor offset for accurate force/torque measurement
        robot_pair.ExecutePrimitive(["ZeroFTSensor", "ZeroFTSensor"], [dict(), dict()])

        # WARNING: during the process, the robots must not contact anything, otherwise the result
        # will be inaccurate and affect following operations
        logger.warning(
            "Zeroing force/torque sensors, make sure nothing is in contact with the robots"
        )

        # Wait for the primitive to finish on both robots
        while not all(st["terminated"] for st in robot_pair.primitive_states()):
            time.sleep(1)
        logger.info("Sensor zeroing complete")

        # Search for Contact
        # =========================================================================================
        # NOTE: there are several ways to do contact search, such as using primitives, or real-time
        # and non-real-time direct motion controls, etc. Here we use non-real-time direct Cartesian
        # control for example.
        logger.info("Searching for contact ...")

        # Set initial poses to current TCP poses in the shared world frame
        left_states, right_states = robot_pair.states()
        init_pose = [left_states.tcp_pose.copy(), right_states.tcp_pose.copy()]
        logger.info(
            f"Initial TCP pose of the left robot set to {init_pose[0]} (position 3x1, rotation "
            "(quaternion) 4x1)"
        )
        logger.info(
            f"Initial TCP pose of the right robot set to {init_pose[1]} (position 3x1, rotation "
            "(quaternion) 4x1)"
        )

        # Use non-real-time mode to make the robots go to a set point with their own motion
        # generators
        robot_pair.SwitchMode(Mode.NRT_CARTESIAN_MOTION_FORCE)

        # Search for contact with max contact wrench set to a small value for making soft contact
        robot_pair.SetMaxContactWrench(
            [MAX_WRENCH_FOR_CONTACT_SEARCH, MAX_WRENCH_FOR_CONTACT_SEARCH]
        )

        # Set target points along -Z direction and expect contact to happen during the travel
        target_pose = [init_pose[0].copy(), init_pose[1].copy()]
        target_pose[0][2] -= SEARCH_DISTANCE
        target_pose[1][2] -= SEARCH_DISTANCE

        # Send target points to both robots to start searching for contact and limit the velocity.
        # Keep target wrench 0 at this stage since we are not doing force control yet
        robot_pair.SendCartesianMotionForce(
            target_pose, max_linear_vel=[SEARCH_VELOCITY, SEARCH_VELOCITY]
        )

        # Use a while loop to poll robot states and check if a contact is made on both robots. A
        # robot that has made contact keeps pressing softly, limited by the max contact wrench,
        # while waiting for the other robot
        is_contacted = [False, False]
        while not all(is_contacted):
            for i, (robot_name, states) in enumerate(
                zip(["left", "right"], robot_pair.states())
            ):
                # Contact is considered to be made if sensed TCP force exceeds the threshold
                ext_force_norm = np.linalg.norm(states.ext_wrench_in_world[:3])
                if not is_contacted[i] and ext_force_norm > PRESSING_FORCE:
                    is_contacted[i] = True
                    logger.info(f"Contact detected at {robot_name} robot TCP")

            # Check at 1ms interval
            time.sleep(0.001)

        # Configure Force Control
        # =========================================================================================
        # Uncomment the following lines to enable passive force control, otherwise active force
        # control is used by default. Passive force control can only be configured in IDLE mode.
        # See function doc for more details
        # robot_pair.SwitchMode(Mode.IDLE)
        # robot_pair.SetPassiveForceControl([True, True])

        # Switch both robots to non-real-time mode for discrete motion force control
        robot_pair.SwitchMode(Mode.NRT_CARTESIAN_MOTION_FORCE)

        # Set force control reference frame based on program argument. See function doc for more
        # details
        robot_pair.SetForceControlFrame([force_ctrl_frame, force_ctrl_frame])

        # Set which Cartesian axis(s) to activate for force control. See function doc for more
        # details. Here we only active Z axis
        force_ctrl_axes = [False, False, True, False, False, False]
        robot_pair.SetForceControlAxis([force_ctrl_axes, force_ctrl_axes])

        # NOTE: motion control always uses the shared world frame, while force control can use
        # either world or TCP frame as reference frame

        # Start Unified Motion Force Control
        # =========================================================================================
        # Disable max contact wrench regulation. Need to do this AFTER the force control in Z axis
        # is activated (i.e. motion control disabled in Z axis) and the motion force control mode
        # is entered, this way the contact force along Z axis is explicitly regulated and will not
        # spike after the max contact wrench regulation for motion control is disabled
        robot_pair.SetMaxContactWrench([[float("inf")] * 6, [float("inf")] * 6])

        # Update initial poses to current TCP poses
        left_states, right_states = robot_pair.states()
        init_pose = [left_states.tcp_pose.copy(), right_states.tcp_pose.copy()]
        logger.info(
            f"Initial TCP pose of the left robot set to {init_pose[0]} (position 3x1, rotation "
            "(quaternion) 4x1)"
        )
        logger.info(
            f"Initial TCP pose of the right robot set to {init_pose[1]} (position 3x1, rotation "
            "(quaternion) 4x1)"
        )

        # Periodic Task
        # =========================================================================================
        # Set loop period
        period = 1.0 / frequency
        logger.info(
            f"Sending command to robots at {frequency} Hz, or {period} seconds interval"
        )

        # Periodic loop counter
        loop_counter = 0

        # Send command periodically at user-specified frequency
        while True:
            # Use sleep to control loop period
            time.sleep(period)

            # Monitor fault on the connected robots
            if robot_pair.fault():
                raise Exception("Fault occurred on the connected robots, exiting ...")

            # Initialize target poses to initial poses
            target_pose = [init_pose[0].copy(), init_pose[1].copy()]

            # Set Fz according to reference frame to achieve a "pressing down" behavior
            Fz = 0.0
            if force_ctrl_frame == CoordType.WORLD:
                Fz = PRESSING_FORCE
            elif force_ctrl_frame == CoordType.TCP:
                Fz = -PRESSING_FORCE
            target_wrench = [0.0, 0.0, Fz, 0.0, 0.0, 0.0]

            # Apply constant force along Z axis of chosen reference frame, and do a simple polish
            # motion along XY plane in the shared world frame
            if args.polish:
                # Create motion commands to sine-sweep along Y direction
                offset = SWING_AMP * math.sin(
                    2 * math.pi * SWING_FREQ * loop_counter * period
                )
                target_pose[0][1] = init_pose[0][1] + offset
                target_pose[1][1] = init_pose[1][1] + offset
                # Command target poses and target wrenches
                robot_pair.SendCartesianMotionForce(
                    target_pose, [target_wrench, target_wrench]
                )
            # Apply constant force along Z axis of chosen reference frame, and hold motions in all
            # other axes
            else:
                # Command initial poses and target wrenches
                robot_pair.SendCartesianMotionForce(
                    init_pose, [target_wrench, target_wrench]
                )

            # Increment loop counter
            loop_counter += 1

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
