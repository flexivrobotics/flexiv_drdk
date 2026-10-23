#!/usr/bin/env python

"""basics6_zero_force_torque_sensors.py

This tutorial zeros the force and torque sensors of both robots, which is a recommended (but not
mandatory) step before any operations that require accurate force/torque measurement.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import logging
import flexivdrdk
from flexivrdk import Mode  # for type definition


def print_tcp_wrenches(robot_pair, logger):
    """
    Print the current TCP force/moment readings of both robots.

    """

    left_states, right_states = robot_pair.states()
    logger.info(f"Left robot: {left_states.ext_wrench_in_world} N-Nm")
    logger.info(f"Right robot: {right_states.ext_wrench_in_world} N-Nm")


def main():
    # Program Setup
    # ==============================================================================================
    # Parse arguments
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        "left_robot_sn",
        help="Serial number of the left robot to connect. Remove any space, e.g. Rizon4s-100001",
    )
    argparser.add_argument(
        "right_robot_sn",
        help="Serial number of the right robot to connect. Remove any space, e.g. Rizon4s-100002",
    )
    args = argparser.parse_args()

    # Define alias
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    logger = logging.getLogger("Example")

    # Print description
    logger.info(
        ">>> Tutorial description <<<\nThis tutorial zeros the force and torque sensors of both "
        "robots, which is a recommended (but not mandatory) step before any operations that "
        "require accurate force/torque measurement.\n"
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

        # Zero Sensors
        # ==========================================================================================
        # Get and print the current TCP force/moment readings
        logger.info(
            "TCP force and moment reading in world frame BEFORE sensor zeroing:"
        )
        print_tcp_wrenches(robot_pair, logger)

        # Run the "ZeroFTSensor" primitive on both robots to automatically zero force and torque
        # sensors
        robot_pair.SwitchMode(Mode.NRT_PRIMITIVE_EXECUTION)
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

        # Get and print the current TCP force/moment readings
        logger.info("TCP force and moment reading in world frame AFTER sensor zeroing:")
        print_tcp_wrenches(robot_pair, logger)

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
