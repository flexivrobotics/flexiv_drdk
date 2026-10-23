#!/usr/bin/env python

"""intermediate5_robot_dynamics.py

This tutorial runs the integrated dynamics engine of both robots to obtain robot Jacobian, mass
matrix, and gravity torques. Also checks reachability of a Cartesian pose.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import logging
import flexivdrdk
from flexivrdk import Mode, Model  # for type definition


def compute_dynamics(model, states, logger):
    """
    Update a robot's dynamics engine with the given robot states, then compute and print the
    gravity vector, mass matrix, and Jacobian.

    """

    # Update robot model in dynamics engine
    model.Update(states.q, states.dtheta)

    # Compute gravity vector
    g = model.g()

    # Compute mass matrix
    M = model.M()

    # Compute Jacobian
    J = model.J("flange")

    # Print result
    logger.info("g = ")
    print(g, flush=True)
    logger.info("M = ")
    print(M, flush=True)
    logger.info("J = ")
    print(J, flush=True)
    print()


def check_reachability(model, states, logger):
    """
    Check reachability of a Cartesian pose based on a robot's current pose.

    """

    pose_to_check = states.tcp_pose.copy()
    pose_to_check[0] += 0.1
    logger.info(f"Checking reachability of Cartesian pose {pose_to_check}")
    result = model.reachable(pose_to_check, states.q, True)
    logger.info(f"Got a result: reachable = {result[0]}, IK solution = {result[1]}")


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
        ">>> Tutorial description <<<\nThis tutorial runs the integrated dynamics engine of both "
        "robots to obtain robot Jacobian, mass matrix, and gravity torques. Also checks "
        "reachability of a Cartesian pose.\n"
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

        # Robot Dynamics
        # ==========================================================================================
        # Initialize dynamics engines. DRDK does not provide a dynamics engine for the robot pair,
        # so a flexivrdk.Model is instantiated for each of the underlying flexivrdk.Robot instances
        left_robot, right_robot = robot_pair.instances()
        left_model = Model(left_robot)
        right_model = Model(right_robot)

        # Step dynamics engines 5 times
        for i in range(5):
            left_states, right_states = robot_pair.states()
            logger.info("Left robot:")
            compute_dynamics(left_model, left_states, logger)
            logger.info("Right robot:")
            compute_dynamics(right_model, right_states, logger)

        # Check reachability of a Cartesian pose based on the current pose of each robot. The
        # dynamics engine works in the robot's own world frame, so the states of the underlying
        # flexivrdk.Robot instances are used here instead of RobotPair.states(), whose TCP poses
        # are expressed in the shared world frame
        logger.info("Left robot:")
        check_reachability(left_model, left_robot.states(), logger)
        logger.info("Right robot:")
        check_reachability(right_model, right_robot.states(), logger)

    except Exception as e:
        logger.error(str(e))


if __name__ == "__main__":
    main()
