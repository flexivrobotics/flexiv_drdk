#!/usr/bin/env python

"""basics1_display_robot_states.py

This tutorial does the very first thing: check connection with a pair of robots and print the
received robot states.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import threading
import spdlog  # pip install spdlog
from flexivrdk import RobotStates  # pip install flexivrdk, for type definition
import flexivdrdk  # pip install flexivdrdk


def print_robot_states(robot_pair, logger, stop_event):
    """
    Print robot states data @ 1Hz.

    """

    while not stop_event.is_set():
        # Print all robot states, round all float values to 2 decimals
        left_robot_states, right_robot_states = robot_pair.states()
        logger.info("Left robot states:")
        # fmt: off
        print("{")
        print(f"q: {['%.2f' % i for i in left_robot_states.q]}",)
        print(f"theta: {['%.2f' % i for i in left_robot_states.theta]}")
        print(f"dq: {['%.2f' % i for i in left_robot_states.dq]}")
        print(f"dtheta: {['%.2f' % i for i in left_robot_states.dtheta]}")
        print(f"tau: {['%.2f' % i for i in left_robot_states.tau]}")
        print(f"tau_des: {['%.2f' % i for i in left_robot_states.tau_des]}")
        print(f"tau_dot: {['%.2f' % i for i in left_robot_states.tau_dot]}")
        print(f"tau_ext: {['%.2f' % i for i in left_robot_states.tau_ext]}")
        print(f"tcp_pose: {['%.2f' % i for i in left_robot_states.tcp_pose]}")
        print(f"tcp_velocity: {['%.2f' % i for i in left_robot_states.tcp_vel]}")
        print(f"flange_pose: {['%.2f' % i for i in left_robot_states.flange_pose]}")
        print(f"ft_sensor_raw: {['%.2f' % i for i in left_robot_states.ft_sensor_raw]}")
        print(f"ext_wrench_in_tcp: {['%.2f' % i for i in left_robot_states.ext_wrench_in_tcp]}")
        print(f"ext_wrench_in_world: {['%.2f' % i for i in left_robot_states.ext_wrench_in_world]}")
        print(f"ext_wrench_in_tcp_raw: {['%.2f' % i for i in left_robot_states.ext_wrench_in_tcp_raw]}")
        print(f"ext_wrench_in_world_raw: {['%.2f' % i for i in left_robot_states.ext_wrench_in_world_raw]}")
        print("}", flush= True)
        logger.info("Right robot states:")
        print("{")
        print(f"q: {['%.2f' % i for i in right_robot_states.q]}",)
        print(f"theta: {['%.2f' % i for i in right_robot_states.theta]}")
        print(f"dq: {['%.2f' % i for i in right_robot_states.dq]}")
        print(f"dtheta: {['%.2f' % i for i in right_robot_states.dtheta]}")
        print(f"tau: {['%.2f' % i for i in right_robot_states.tau]}")
        print(f"tau_des: {['%.2f' % i for i in right_robot_states.tau_des]}")
        print(f"tau_dot: {['%.2f' % i for i in right_robot_states.tau_dot]}")
        print(f"tau_ext: {['%.2f' % i for i in right_robot_states.tau_ext]}")
        print(f"tcp_pose: {['%.2f' % i for i in right_robot_states.tcp_pose]}")
        print(f"tcp_velocity: {['%.2f' % i for i in right_robot_states.tcp_vel]}")
        print(f"flange_pose: {['%.2f' % i for i in right_robot_states.flange_pose]}")
        print(f"ft_sensor_raw: {['%.2f' % i for i in right_robot_states.ft_sensor_raw]}")
        print(f"ext_wrench_in_tcp: {['%.2f' % i for i in right_robot_states.ext_wrench_in_tcp]}")
        print(f"ext_wrench_in_world: {['%.2f' % i for i in right_robot_states.ext_wrench_in_world]}")
        print(f"ext_wrench_in_tcp_raw: {['%.2f' % i for i in right_robot_states.ext_wrench_in_tcp_raw]}")
        print(f"ext_wrench_in_world_raw: {['%.2f' % i for i in right_robot_states.ext_wrench_in_world_raw]}")
        print("}", flush= True)
        # fmt: on

        # Print digital inputs
        left_din, right_din = robot_pair.digital_inputs()
        logger.info("Left robot digital inputs:")
        print(left_din)
        logger.info("Right robot digital inputs:")
        print(right_din)
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
    args = argparser.parse_args()

    # Define alias
    logger = spdlog.ConsoleLogger("Example")

    # Print description
    logger.info(
        ">>> Tutorial description <<<\nThis tutorial does the very first thing: check connection "
        "with a pair of robots and print received robot states.\n"
    )

    try:
        # RDK Initialization
        # ==========================================================================================
        # Instantiate robot pair interface
        robot_pair = flexivdrdk.RobotPair([args.left_robot_sn, args.right_robot_sn])

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

        # Enable the pair of robots, make sure the E-stop is released before enabling
        logger.info("Enabling robots ...")
        robot_pair.Enable()

        # Wait for the robot to become operational
        while not robot_pair.operational():
            time.sleep(1)

        logger.info("Both robots are now operational")

    except Exception as e:
        # Print exception error message
        logger.error(str(e))

    # Print States
    # =============================================================================
    # Thread for printing robot states
    print_thread = threading.Thread(
        target=print_robot_states, args=[robot_pair, logger, stop_event]
    )
    print_thread.start()

    # Use main thread to catch keyboard interrupt and exit thread
    try:
        while not stop_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        # Send signal to exit thread
        logger.info("Stopping print thread")
        stop_event.set()

    # Wait for thread to exit
    print_thread.join()
    logger.info("Print thread exited")


if __name__ == "__main__":
    main()
