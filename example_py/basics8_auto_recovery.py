#!/usr/bin/env python

"""basics8_auto_recovery.py

This tutorial runs an automatic recovery process if the safety system of either robot is in
recovery state. See RobotPair::recovery() and RDK manual for more details.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import logging
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
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    logger = logging.getLogger("Example")

    # Print description
    logger.info(
        ">>> Tutorial description <<<\nThis tutorial runs an automatic recovery process if the "
        "safety system of either robot is in recovery state. See "
        "flexiv::drdk::RobotPair::recovery() and RDK manual for more details.\n"
    )

    try:
        # DRDK Initialization
        # ==========================================================================================
        # Instantiate robot pair interface
        robot_pair = flexivdrdk.RobotPair([args.left_robot_sn, args.right_robot_sn])

        # Enable the pair of robots, make sure the E-stop is released before enabling
        logger.info("Enabling robots ...")
        robot_pair.Enable()

        # Run Auto-recovery
        # ==========================================================================================
        # If the system is in recovery state, we can't use operational() to tell if the enabling
        # process is done, so just wait long enough for the process to finish
        time.sleep(8)

        # Run automatic recovery if either robot is in recovery state, the involved joints will
        # start to move back into allowed position range
        left_recovery, right_recovery = robot_pair.recovery()
        if left_recovery or right_recovery:
            logger.info(f"Left robot in recovery state: {left_recovery}")
            logger.info(f"Right robot in recovery state: {right_recovery}")
            robot_pair.RunAutoRecovery()
        # Otherwise both systems are normal, do nothing
        else:
            logger.info(
                "Neither robot is in recovery state, nothing to be done, exiting ..."
            )

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
