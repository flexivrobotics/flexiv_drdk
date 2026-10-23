#!/usr/bin/env python

"""basics10_global_variables.py

This tutorial shows how to get and set global variables of both robots.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import logging
import flexivdrdk
from flexivrdk import Coord  # for type definition


def print_global_variables(global_vars):
    """
    Print names and values of a robot's global variables.

    """

    for key, value in global_vars.items():
        print(f"{key}: {value}")


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
        ">>> Tutorial description <<<\nThis tutorial shows how to get and set global variables "
        "of both robots.\n"
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

        # Get existing global variables
        # =========================================================================================
        left_global_vars, right_global_vars = robot_pair.global_variables()
        if not left_global_vars or not right_global_vars:
            logger.warning("No global variables available on one of the robots")
            return 1
        else:
            logger.info(
                "Existing global variables and their original values of the left robot:"
            )
            print_global_variables(left_global_vars)
            logger.info(
                "Existing global variables and their original values of the right robot:"
            )
            print_global_variables(right_global_vars)

        # Set global variables
        # =========================================================================================
        # WARNING: These specified global variables need to be created first using Flexiv Elements
        new_global_vars = {
            "test_bool": 1,
            "test_int": 100,
            "test_double": 100.123,
            "test_string": "Flexiv",
            "test_int_vec": [1, 2, 3],
            "test_double_vec": [1.1, 2.2, 3.3],
            "test_string_vec": ["Go", "Flexiv", "Go!"],
            "test_pose": [0.1, -0.2, 0.3, -90, -45, 120],
            "test_coord": Coord(
                [0.1, -0.2, 0.3],
                [-90, -45, 120],
                ["WORLD", "WORLD_ORIGIN"],
                [1, 2, 3, 4, 5, 6, 7],
                [10, 20, 0, 0, 0, 0],
            ),
            "test_coord_array": [
                Coord([1, 2, 3], [4, 5, 6], ["WORK", "WorkCoord0"]),
                Coord(
                    [10, 20, 30],
                    [40, 50, 60],
                    ["WORLD", "WORLD_ORIGIN"],
                    [1, 2, 3, 4, 5, 6, 7],
                    [10, 20, 0, 0, 0, 0],
                ),
                Coord([3, 2, 1], [180, 0, 180], ["WORLD", "WORLD_ORIGIN"]),
            ],
        }

        # The global variables are given respectively for the left and right robot. Here both
        # robots use the same values, but they can be different
        logger.info("Setting new values to existing global variables")
        robot_pair.SetGlobalVariables([new_global_vars, new_global_vars])

        # Get updated global variables
        # =========================================================================================
        left_global_vars, right_global_vars = robot_pair.global_variables()
        if not left_global_vars or not right_global_vars:
            logger.warning("No global variables available on one of the robots")
            return 1
        else:
            logger.info("Updated global variables of the left robot:")
            print_global_variables(left_global_vars)
            logger.info("Updated global variables of the right robot:")
            print_global_variables(right_global_vars)

        logger.info("Program finished")

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
