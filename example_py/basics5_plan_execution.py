#!/usr/bin/env python

"""basics5_plan_execution.py

This tutorial executes plans selected by the user from the lists of available plans of both robots.
A plan is a pre-written script to execute a series of robot primitives with pre-defined transition
conditions between 2 adjacent primitives. Users can use Flexiv Elements to compose their own plan
and assign to the robot, which will appear in the plan list.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import logging
import flexivdrdk
from flexivrdk import Mode  # for type definition


def print_plan_info(robot_pair, logger):
    """
    Print plan info of both robots @ 1Hz while any of them is busy.

    """

    while robot_pair.busy():
        for robot_name, plan_info in zip(["left", "right"], robot_pair.plan_info()):
            logger.info(f"Current plan info of the {robot_name} robot:")
            print(f"assigned_plan_name: {plan_info.assigned_plan_name}")
            print(f"pt_name: {plan_info.pt_name}")
            print(f"node_name: {plan_info.node_name}")
            print(f"node_path: {plan_info.node_path}")
            print(f"node_path_time_period: {plan_info.node_path_time_period}")
            print(f"node_path_number: {plan_info.node_path_number}")
            print(f"velocity_scale: {plan_info.velocity_scale}")
            print(f"waiting_for_step: {plan_info.waiting_for_step}")
            print("", flush=True)
        time.sleep(1)


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
        ">>> Tutorial description <<<\nThis tutorial executes plans selected by the user from the "
        "lists of available plans of both robots. A plan is a pre-written script to execute a "
        "series of robot primitives with pre-defined transition conditions between 2 adjacent "
        "primitives. Users can use Flexiv Elements to compose their own plan and assign to the "
        "robot, which will appear in the plan list.\n"
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

        # Execute Plans
        # ==========================================================================================
        # Switch both robots to plan execution mode
        robot_pair.SwitchMode(Mode.NRT_PLAN_EXECUTION)

        while True:
            # Monitor fault on the connected robots
            if robot_pair.fault():
                raise Exception("Fault occurred on the connected robots, exiting ...")

            # Get user input
            logger.info("Choose an action:")
            print("[1] Show available plans")
            print("[2] Execute plans by index")
            print("[3] Execute plans by name")
            user_input = int(input())

            # Get and show plan lists
            if user_input == 1:
                left_plan_list, right_plan_list = robot_pair.plan_list()
                logger.info("Available plans of the left robot:")
                for i in range(len(left_plan_list)):
                    print(f"[{i}] {left_plan_list[i]}")
                logger.info("Available plans of the right robot:")
                for i in range(len(right_plan_list)):
                    print(f"[{i}] {right_plan_list[i]}")
                print("")

            # Execute plans by index
            elif user_input == 2:
                left_index = int(
                    input("Enter plan index to execute on the left robot:\n")
                )
                right_index = int(
                    input("Enter plan index to execute on the right robot:\n")
                )
                # Allow the plans to continue their execution even if the DRDK program is closed
                # or the connection is lost
                robot_pair.ExecutePlan([left_index, right_index], True)

                # Print plan info while the current plans are running
                print_plan_info(robot_pair, logger)

            # Execute plans by name
            elif user_input == 3:
                left_name = str(
                    input("Enter plan name to execute on the left robot:\n")
                )
                right_name = str(
                    input("Enter plan name to execute on the right robot:\n")
                )
                # Allow the plans to continue their execution even if the DRDK program is closed
                # or the connection is lost
                robot_pair.ExecutePlan([left_name, right_name], True)

                # Print plan info while the current plans are running
                print_plan_info(robot_pair, logger)

            else:
                logger.warning("Invalid input")

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
