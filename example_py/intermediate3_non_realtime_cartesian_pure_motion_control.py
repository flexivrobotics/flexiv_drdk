#!/usr/bin/env python

"""intermediate3_non_realtime_cartesian_pure_motion_control.py

This tutorial runs non-real-time Cartesian-space pure motion control to hold or sine-sweep the TCPs
of both robots. A simple collision detection is also included.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import math
import argparse
import logging
import numpy as np  # pip install numpy
import flexivdrdk
from flexivrdk import Mode  # for type definition

# Global constants
# ==================================================================================================
# TCP sine-sweep amplitude [m]
SWING_AMP = 0.1

# TCP sine-sweep frequency [Hz]
SWING_FREQ = 0.3

# External TCP force threshold for collision detection, value is only for demo purpose [N]
EXT_FORCE_THRESHOLD = 10.0

# External joint torque threshold for collision detection, value is only for demo purpose [Nm]
EXT_TORQUE_THRESHOLD = 5.0


def collision_detected(states):
    """
    Simple collision detection based on the external force at TCP and the external joint torques
    of a robot.

    """

    if np.linalg.norm(states.ext_wrench_in_world[:3]) > EXT_FORCE_THRESHOLD:
        return True
    return any(abs(v) > EXT_TORQUE_THRESHOLD for v in states.tau_ext)


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
        "--hold",
        action="store_true",
        help="Robots hold current TCP pose, otherwise do a sine-sweep",
    )
    argparser.add_argument(
        "--collision",
        action="store_true",
        help="Enable collision detection, robots will stop upon collision",
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
        ">>> Tutorial description <<<\nThis tutorial runs non-real-time Cartesian-space pure "
        "motion control to hold or sine-sweep the TCPs of both robots. A simple collision "
        "detection is also included.\n"
    )

    # Print based on arguments
    if args.hold:
        logger.info("Robots holding current TCP pose")
    else:
        logger.info("Robots running TCP sine-sweep")

    if args.collision:
        logger.info("Collision detection enabled")
    else:
        logger.info("Collision detection disabled")

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

        # Configure Motion Control
        # =========================================================================================
        # The Cartesian motion force modes do pure motion control out of the box, thus nothing
        # needs to be explicitly configured

        # NOTE: motion control always uses the shared world frame, while force control can use
        # either world or TCP frame as reference frame

        # Start Pure Motion Control
        # =========================================================================================
        # Switch both robots to non-real-time mode for discrete motion control
        robot_pair.SwitchMode(Mode.NRT_CARTESIAN_MOTION_FORCE)

        # Set all Cartesian axis(s) of both robots to motion control
        robot_pair.SetForceControlAxis([[False] * 6, [False] * 6])

        # Save initial poses in the shared world frame and initial joint positions
        left_states, right_states = robot_pair.states()
        init_pose = [left_states.tcp_pose.copy(), right_states.tcp_pose.copy()]
        init_q = [left_states.q.copy(), right_states.q.copy()]

        # Periodic Task
        # =========================================================================================
        # Set loop period
        period = 1.0 / frequency
        loop_counter = 0
        logger.info(
            f"Sending command to robots at {frequency} Hz, or {period} seconds interval"
        )

        # Send command periodically at user-specified frequency
        while True:
            # Use sleep to control loop period
            time.sleep(period)

            # Monitor fault on the connected robots
            if robot_pair.fault():
                raise Exception("Fault occurred on the connected robots, exiting ...")

            # Initialize target poses to initial poses
            target_pose = [init_pose[0].copy(), init_pose[1].copy()]

            # Sine-sweep TCPs of both robots along Y axis of the shared world frame
            if not args.hold:
                offset = SWING_AMP * math.sin(
                    2 * math.pi * SWING_FREQ * loop_counter * period
                )
                target_pose[0][1] = init_pose[0][1] + offset
                target_pose[1][1] = init_pose[1][1] + offset
            # Otherwise robot TCPs will hold at initial poses

            # Send commands. Calling this method with only target pose input results
            # in pure motion control
            robot_pair.SendCartesianMotionForce(target_pose)

            # Do the following operations in sequence for every 20 seconds
            loop_in_cycle = loop_counter % (20 * frequency)
            # Online change reference joint positions at 3 seconds
            if loop_in_cycle == 3 * frequency:
                preferred_jnt_pos = [0.938, -1.108, -1.254, 1.464, 1.073, 0.278, -0.658]
                robot_pair.SetNullSpacePosture([preferred_jnt_pos, preferred_jnt_pos])
                logger.info(
                    f"Reference joint positions of both robots set to {preferred_jnt_pos}"
                )
            # Online change stiffness to half of nominal at 6 seconds
            elif loop_in_cycle == 6 * frequency:
                new_K = [[k * 0.5 for k in info.K_x_nom] for info in robot_pair.info()]
                robot_pair.SetCartesianImpedance(new_K)
                logger.info(f"Cartesian stiffness of the left robot set to {new_K[0]}")
                logger.info(f"Cartesian stiffness of the right robot set to {new_K[1]}")
            # Online change to another reference joint positions at 9 seconds
            elif loop_in_cycle == 9 * frequency:
                preferred_jnt_pos = [-0.938, -1.108, 1.254, 1.464, -1.073, 0.278, 0.658]
                robot_pair.SetNullSpacePosture([preferred_jnt_pos, preferred_jnt_pos])
                logger.info(
                    f"Reference joint positions of both robots set to {preferred_jnt_pos}"
                )
            # Online reset impedance properties to nominal at 12 seconds
            elif loop_in_cycle == 12 * frequency:
                left_info, right_info = robot_pair.info()
                robot_pair.SetCartesianImpedance(
                    [left_info.K_x_nom, right_info.K_x_nom]
                )
                logger.info("Cartesian impedance properties are reset")
            # Online reset reference joint positions to nominal at 14 seconds
            elif loop_in_cycle == 14 * frequency:
                robot_pair.SetNullSpacePosture(init_q)
                logger.info("Reference joint positions are reset")
            # Online enable max contact wrench regulation at 16 seconds
            elif loop_in_cycle == 16 * frequency:
                max_wrench = [10.0, 10.0, 10.0, 2.0, 2.0, 2.0]
                robot_pair.SetMaxContactWrench([max_wrench, max_wrench])
                logger.info(f"Max contact wrench of both robots set to {max_wrench}")
            # Disable max contact wrench regulation at 19 seconds
            elif loop_in_cycle == 19 * frequency:
                robot_pair.SetMaxContactWrench([[float("inf")] * 6, [float("inf")] * 6])
                logger.info("Max contact wrench regulation is disabled")

            # Simple collision detection: stop both robots if collision is detected at the
            # end-effector or robot body of either robot
            if args.collision:
                if any(collision_detected(st) for st in robot_pair.states()):
                    robot_pair.Stop()
                    logger.warning(
                        "Collision detected, stopping robots and exit program ..."
                    )
                    return

            # Increment loop counter
            loop_counter += 1

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
