#!/usr/bin/env python

"""basics4_primitive_execution.py

This tutorial executes several basic robot primitives (unit skills) simultaneously on both robots
in the pair. For detailed documentation on all available primitives, please see [Flexiv
Primitives](https://www.flexiv.com/primitives/).
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import logging
import flexivdrdk
from flexivrdk import Mode, JPos, Coord  # for type definition
from utility import quat2eulerZYX


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
        ">>> Tutorial description <<<\nThis tutorial executes several basic robot primitives (unit "
        "skills) simultaneously on both robots in the pair. For detailed documentation on all "
        "available primitives, please see [Flexiv "
        "Primitives](https://www.flexiv.com/primitives/).\n"
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

        # Execute Primitives
        # ==========================================================================================
        # Switch both robots to primitive execution mode
        robot_pair.SwitchMode(Mode.NRT_PRIMITIVE_EXECUTION)

        # (1) Go to home pose
        # ------------------------------------------------------------------------------------------
        # All parameters of the "Home" primitive are optional, thus we can skip the parameters and
        # the default values will be used
        logger.info("Executing primitive: Home")

        # Send command to both robots
        robot_pair.ExecutePrimitive(["Home", "Home"], [dict(), dict()])

        # Wait for both robots to reach target
        # Note: primitive_states() returns a dictionary of {pt_state_name, [pt_state_values]} for
        # each robot
        while not all(st["reachedTarget"] for st in robot_pair.primitive_states()):
            time.sleep(1)

        # (2) Move robot joints to target positions
        # ------------------------------------------------------------------------------------------
        # Required parameters:
        #     target: final joint positions, unit: degrees
        #         {arm joint positions}, {external axis joint positions, optional}
        # Optional parameters:
        #     waypoints: waypoints to pass before reaching the target
        #         (same format as above, but can be more than one)
        #     vel: TCP linear velocity, unit: m/s
        logger.info("Executing primitive: MoveJ")

        # The input parameters are given respectively for the left and right robot. Here both
        # robots use the same parameters, but they can be different
        move_j_params = {
            "target": JPos([30, -45, 0, 90, 0, 40, 30], [-50, 30, 0, 0, 0, 0]),
            "waypoints": [
                JPos([10, -30, 10, 30, 10, 15, 10], [-15, 10, 0, 0, 0, 0]),
                JPos([20, -60, -10, 60, -10, 30, 20], [-30, 20, 0, 0, 0, 0]),
            ],
        }

        # Send command to both robots
        robot_pair.ExecutePrimitive(["MoveJ", "MoveJ"], [move_j_params, move_j_params])

        # Most primitives won't exit by themselves and require users to explicitly trigger
        # transitions based on specific primitive states. Here we check if the primitive state
        # [reachedTarget] becomes true on both robots and trigger the transition manually by
        # sending a new primitive command.
        while not all(st["reachedTarget"] for st in robot_pair.primitive_states()):
            # Print current primitive states of both robots
            left_pt_states, right_pt_states = robot_pair.primitive_states()
            logger.info("Current primitive states of the left robot:")
            print(left_pt_states)
            logger.info("Current primitive states of the right robot:")
            print(right_pt_states, flush=True)
            time.sleep(1)

        # (3) Move robot TCP to a target pose in world (base) frame
        # ------------------------------------------------------------------------------------------
        # Required parameters:
        #     target: final TCP pose, unit: m and degrees
        #         {pos_x, pos_y, pos_z}, {rot_x, rot_y, rot_z}, {ref_frame, ref_point}
        # Optional parameters:
        #     waypoints: waypoints to pass before reaching the target
        #         (same format as above, but can be more than one)
        #     vel: TCP linear velocity, unit: m/s
        # NOTE: The rotations use Euler ZYX convention, rot_x means Euler ZYX angle around X axis
        # NOTE: The WORLD frame used by primitives is each robot's own world frame, not the shared
        # world frame of the robot pair
        logger.info("Executing primitive: MoveL")

        # Input parameters of the primitive, used by both robots
        move_l_params = {
            "target": Coord(
                [0.65, -0.3, 0.2], [180, 0, 180], ["WORLD", "WORLD_ORIGIN"]
            ),
            "waypoints": [
                Coord([0.45, 0.1, 0.2], [180, 0, 180], ["WORLD", "WORLD_ORIGIN"]),
                Coord([0.45, -0.3, 0.2], [180, 0, 180], ["WORLD", "WORLD_ORIGIN"]),
            ],
            "vel": 0.6,
            "zoneRadius": "Z50",
        }

        # Send command to both robots
        robot_pair.ExecutePrimitive(["MoveL", "MoveL"], [move_l_params, move_l_params])

        # Wait for both robots to reach target
        while not all(st["reachedTarget"] for st in robot_pair.primitive_states()):
            time.sleep(1)

        # (4) Another MoveL that uses TCP frame
        # ------------------------------------------------------------------------------------------
        # In this example the reference frame is changed from WORLD::WORLD_ORIGIN to TRAJ::START,
        # which represents the current TCP frame
        logger.info("Executing primitive: MoveL")

        # Example to convert target quaternion [w,x,y,z] to Euler ZYX using scipy package's 'xyz'
        # extrinsic rotation
        target_quat = [0.9185587, 0.1767767, 0.3061862, 0.1767767]
        # ZYX = [30, 30, 30] degrees
        eulerZYX_deg = quat2eulerZYX(target_quat, degree=True)

        # Input parameters of the primitive, used by both robots. This motion will hold current TCP
        # position and only do rotation
        rotate_params = {
            "target": Coord([0.0, 0.0, 0.0], eulerZYX_deg, ["TRAJ", "START"]),
            "vel": 0.2,
        }

        # Send command to both robots
        robot_pair.ExecutePrimitive(["MoveL", "MoveL"], [rotate_params, rotate_params])

        # Wait for both robots to reach target
        while not all(st["reachedTarget"] for st in robot_pair.primitive_states()):
            time.sleep(1)

        # All done, stop both robots and put into IDLE mode
        robot_pair.Stop()

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
