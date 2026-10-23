#!/usr/bin/env python

"""basics7_gripper_control.py

This tutorial simultaneously does position and force (if available) control of a pair of grippers
supported by Flexiv, which are installed respectively on the pair of robots.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import sys
import time
import argparse
import threading
import logging
import flexivdrdk
from flexivrdk import Tool  # for type definition


def print_gripper_params(params):
    """
    Print parameters of a gripper.

    """

    print(f"name: {params.name}")
    print(f"min_width: {round(params.min_width, 3)}")
    print(f"max_width: {round(params.max_width, 3)}")
    print(f"min_force: {round(params.min_force, 3)}")
    print(f"max_force: {round(params.max_force, 3)}")
    print(f"min_vel: {round(params.min_vel, 3)}")
    print(f"max_vel: {round(params.max_vel, 3)}")
    print("", flush=True)


def print_gripper_states(gripper_pair, logger, stop_event):
    """
    Print states data of both grippers @ 1Hz.

    """

    while not stop_event.is_set():
        # Print all gripper states, round all float values to 2 decimals
        for gripper_name, states in zip(["left", "right"], gripper_pair.states()):
            logger.info(f"Current states of the {gripper_name} gripper:")
            print(f"width: {round(states.width, 2)}")
            print(f"force: {round(states.force, 2)}")
            print(f"is_moving: {states.is_moving}")
            print("", flush=True)
        time.sleep(1)


def main():
    # Create an event to signal the thread to stop
    stop_event = threading.Event()

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
        "left_gripper_name",
        help="Full name of the gripper installed on the left robot, can be found in Flexiv Elements -> Settings -> Device",
    )
    argparser.add_argument(
        "right_gripper_name",
        help="Full name of the gripper installed on the right robot, can be found in Flexiv Elements -> Settings -> Device",
    )
    args = argparser.parse_args()

    # Define alias
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    logger = logging.getLogger("Example")

    # Print description
    logger.info(
        ">>> Tutorial description <<<\nThis tutorial simultaneously does position and force (if "
        "available) control of a pair of grippers supported by Flexiv, which are installed "
        "respectively on the pair of robots.\n"
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

        # Gripper Control
        # ==========================================================================================
        # Instantiate gripper pair control interface
        gripper_pair = flexivdrdk.GripperPair(robot_pair)

        # Instantiate tool interfaces. Gripper is categorized as both a device and a tool. The
        # device attribute allows a gripper to be interactively controlled by the user; whereas the
        # tool attribute tells the robot to account for its mass properties and TCP location. DRDK
        # does not provide a tool interface for the robot pair, so a flexivrdk.Tool interface is
        # instantiated for each of the underlying flexivrdk.Robot instances
        left_robot, right_robot = robot_pair.instances()
        left_tool = Tool(left_robot)
        right_tool = Tool(right_robot)

        # Enable the specified grippers as devices. This is equivalent to enabling the specified
        # grippers in Flexiv Elements -> Settings -> Device
        logger.info(
            f"Enabling grippers [{args.left_gripper_name}] and [{args.right_gripper_name}]"
        )
        gripper_pair.Enable([args.left_gripper_name, args.right_gripper_name])

        # Print parameters of the enabled grippers
        left_params, right_params = gripper_pair.params()
        logger.info("Left gripper params:")
        print_gripper_params(left_params)
        logger.info("Right gripper params:")
        print_gripper_params(right_params)

        # Switch robot tools to grippers so the gravity compensation and TCP location is updated
        logger.info(
            f"Switching tools of the left and right robot to [{args.left_gripper_name}] and "
            f"[{args.right_gripper_name}]"
        )
        left_tool.Switch(args.left_gripper_name)
        right_tool.Switch(args.right_gripper_name)

        # User needs to determine if the grippers require manual initialization
        logger.info(
            "Manually trigger initialization for the grippers now? Choose Yes if they are 48v "
            "Grav grippers"
        )
        print("[1] No, they have already initialized automatically when power on")
        print("[2] Yes, they do not initialize themselves when power on")
        choice = int(input(""))

        # Trigger manual initialization based on input
        if choice == 1:
            logger.info("Skipped manual initialization")
        elif choice == 2:
            gripper_pair.Init()
            # User determines if the manual initialization is finished
            logger.info(
                "Triggered manual initialization, press Enter when the initialization is finished to continue"
            )
            input()
        else:
            logger.error("Invalid choice")
            return 1

        # Start a separate thread to print gripper states
        print_thread = threading.Thread(
            target=print_gripper_states, args=[gripper_pair, logger, stop_event]
        )
        print_thread.start()

        # Position control
        logger.info("Closing grippers")
        gripper_pair.Move([0.01, 0.01], [0.1, 0.1], [20, 20])
        time.sleep(2)
        logger.info("Opening grippers")
        gripper_pair.Move([0.09, 0.09], [0.1, 0.1], [20, 20])
        time.sleep(2)

        # Stop
        logger.info("Closing grippers")
        gripper_pair.Move([0.01, 0.01], [0.1, 0.1], [20, 20])
        time.sleep(0.5)
        logger.info("Stopping grippers")
        gripper_pair.Stop()
        time.sleep(2)
        logger.info("Closing grippers")
        gripper_pair.Move([0.01, 0.01], [0.1, 0.1], [20, 20])
        time.sleep(2)
        logger.info("Opening grippers")
        gripper_pair.Move([0.09, 0.09], [0.1, 0.1], [20, 20])
        time.sleep(0.5)
        logger.info("Stopping grippers")
        gripper_pair.Stop()
        time.sleep(2)

        # Force control, if available on both grippers (sensed force is not zero)
        if all(abs(st.force) > sys.float_info.epsilon for st in gripper_pair.states()):
            logger.info("Grippers running zero force control")
            gripper_pair.Grasp([0, 0])
            # Exit after 10 seconds
            time.sleep(10)

        # Finished
        gripper_pair.Stop()

        # Stop all threads
        logger.info("Stopping print thread")
        stop_event.set()
        print_thread.join()
        logger.info("Print thread exited")
        logger.info("Program finished")

    except Exception as e:
        logger.error(str(e))


if __name__ == "__main__":
    main()
