#!/usr/bin/env python

"""basics2_clear_fault.py

This tutorial clears minor or critical faults, if any, of the connected robots.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import argparse
import spdlog
import flexivdrdk


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
    logger = spdlog.ConsoleLogger("Example")

    # Print description
    logger.info(
        ">>> Tutorial description <<<\nThis tutorial clears minor or critical faults, if any, of "
        "the connected robots.\n"
    )

    try:
        # DRDK Initialization
        # ==========================================================================================
        # Instantiate robot pair interface
        robot_pair = flexivdrdk.RobotPair([args.left_robot_sn, args.right_robot_sn])

        # Fault Clearing
        # ==========================================================================================
        # Clear fault on the connected robot if any
        if robot_pair.fault():
            logger.warn("Fault occurred on the connected robot, trying to clear ...")
            # Try to clear the fault on both robots
            result = robot_pair.ClearFault()
            # If fault is not cleared on both robots
            if not (result[0] and result[1]):
                logger.error("Fault cannot be cleared, exiting ...")
                return 1
            logger.info("Fault on the connected robot is cleared")
        else:
            logger.info("No fault on the connected robot")

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
