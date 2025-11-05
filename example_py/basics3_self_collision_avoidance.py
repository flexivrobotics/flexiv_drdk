#!/usr/bin/env python

"""basics3_self_collision_avoidance.py

This tutorial shows how to use the self collision monitor to keep the left and right robot from
hitting each other.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import spdlog
import flexivdrdk
from flexivrdk import Mode  # for type definition


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
    argparser.add_argument(
        "left_robot_urdf",
        help="Absolute path to the URDF of the left robot",
    )
    argparser.add_argument(
        "right_robot_urdf",
        help="Absolute path to the URDF of the right robot",
    )
    args = argparser.parse_args()

    # Define alias
    logger = spdlog.ConsoleLogger("Example")

    # Print description
    logger.info(
        ">>> Tutorial description <<<\nThis tutorial shows how to use the self collision monitor "
        "to keep the left and right robot from hitting each other.\n"
    )

    try:
        # DRDK Initialization
        # ==========================================================================================
        # Translation in world of the left and robot
        left_trans_in_world = [0, 0.3, 0]
        right_trans_in_world = [0, -0.3, 0]

        # Instantiate robot pair interface
        robot_pair = flexivdrdk.RobotPair(
            [args.left_robot_sn, args.right_robot_sn],
            [left_trans_in_world, right_trans_in_world],
        )

        # Self Collision Monitor
        # ==========================================================================================
        # Instantiate self collision monitor interface
        coll_monitor = flexivdrdk.SelfCollisionMonitor(
            robot_pair, [args.left_robot_urdf, args.right_robot_urdf]
        )

        # Use 10 cm safety distance, leaving some buffer for the robots to decelerate
        coll_monitor.SetMinDistance(0.1)
        logger.info("Safety distance set to 10 cm")

        # Start collision monitor and keep it running in the background
        coll_monitor.Start()
        logger.info("Self-collision monitor started")

        # Move robots to home pose
        robot_pair.SwitchMode(Mode.NRT_PLAN_EXECUTION)
        robot_pair.ExecutePlan(["PLAN-Home", "PLAN-Home"])
        while robot_pair.busy():
            time.sleep(1)
        logger.info("Robot pair reached home")

        # Move the two robots to the same Cartesian location in world
        logger.info("Moving the robots towards each other")
        robot_pair.SwitchMode(Mode.NRT_CARTESIAN_MOTION_FORCE)
        target_pose = [0.5, 0.0, 0.3, 0, 0, 1, 0]
        robot_pair.SendCartesianMotionForce([target_pose, target_pose])

        # Both robots will stop before they hit each other
        time.sleep(1)
        while not robot_pair.stopped():
            time.sleep(1)

        logger.info("Program finished")

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
