#!/usr/bin/env python

"""intermediate2_non_realtime_joint_impedance_control.py

This tutorial runs non-real-time joint impedance control to hold or sine-sweep all joints of both
robots.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import math
import argparse
import logging
import flexivdrdk
from flexivrdk import Mode  # for type definition


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
        help="Robots hold current joint positions, otherwise do a sine-sweep",
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
        ">>> Tutorial description <<<\nThis tutorial runs non-real-time joint impedance control to "
        "hold or sine-sweep all joints of both robots.\n"
    )

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

        # Non-real-time Joint Impedance Control
        # ==========================================================================================
        # Switch both robots to non-real-time joint impedance control mode
        robot_pair.SwitchMode(Mode.NRT_JOINT_IMPEDANCE)

        period = 1.0 / frequency
        loop_counter = 0
        logger.info(
            f"Sending command to robots at {frequency} Hz, or {period} seconds interval"
        )

        # Use current joint positions of both robots as initial positions
        left_states, right_states = robot_pair.states()
        init_pos = [left_states.q.copy(), right_states.q.copy()]
        logger.info(f"Initial positions of the left robot set to: {init_pos[0]}")
        logger.info(f"Initial positions of the right robot set to: {init_pos[1]}")

        # Joint degrees of freedom of both robots
        left_info, right_info = robot_pair.info()
        DoF = [left_info.DoF, right_info.DoF]

        # Initialize target vectors
        target_pos = [init_pos[0].copy(), init_pos[1].copy()]
        target_vel = [[0.0] * DoF[0], [0.0] * DoF[1]]

        # Joint motion constraints
        MAX_VEL = [[2.0] * DoF[0], [2.0] * DoF[1]]
        MAX_ACC = [[3.0] * DoF[0], [3.0] * DoF[1]]

        # Joint sine-sweep amplitude [rad]
        SWING_AMP = 0.1

        # Joint sine-sweep frequency [Hz]
        SWING_FREQ = 0.3

        # Send command periodically at user-specified frequency
        while True:
            # Use sleep to control loop period
            time.sleep(period)

            # Monitor fault on the connected robots
            if robot_pair.fault():
                raise Exception("Fault occurred on the connected robots, exiting ...")

            # Sine-sweep all joints of both robots
            if not args.hold:
                offset = SWING_AMP * math.sin(
                    2 * math.pi * SWING_FREQ * loop_counter * period
                )
                target_pos = [[q + offset for q in pos] for pos in init_pos]
            # Otherwise all joints will hold at initial positions

            # Reduce stiffness of both robots to half of nominal values after 5 seconds
            if loop_counter == 5 * frequency:
                new_Kq = [[k * 0.5 for k in info.K_q_nom] for info in robot_pair.info()]
                robot_pair.SetJointImpedance(new_Kq)
                logger.info(f"Joint stiffness of the left robot set to {new_Kq[0]}")
                logger.info(f"Joint stiffness of the right robot set to {new_Kq[1]}")

            # Reset impedance properties of both robots to nominal values after another 5 seconds
            if loop_counter == 10 * frequency:
                left_info, right_info = robot_pair.info()
                robot_pair.SetJointImpedance([left_info.K_q_nom, right_info.K_q_nom])
                logger.info("Joint impedance properties are reset")

            # Send commands
            robot_pair.SendJointPosition(target_pos, target_vel, MAX_VEL, MAX_ACC)

            # Increment loop counter
            loop_counter += 1

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
