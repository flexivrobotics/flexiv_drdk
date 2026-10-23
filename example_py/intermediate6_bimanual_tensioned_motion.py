#!/usr/bin/env python

"""intermediate6_bimanual_tensioned_motion.py

This tutorial uses the bimanual primitive MoveTensioned to squeeze an object between the
end-effectors of the two robots, then lift it up and put it back down while keeping the squeezing
force. The two robots are assumed to be mounted upright, side by side along the Y axis of the
shared world frame. Modify the base translations and holding poses below to match the actual setup.

WARNING: an object must be held between the two end-effectors before the squeezing starts,
otherwise the two robots will keep moving towards each other.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import logging
import flexivdrdk
from flexivrdk import Mode  # for type definition

# Global constants
# ==================================================================================================
# Translations of the left and right robot's base frame in the shared world frame [m]
LEFT_TRANS_IN_WORLD = [0.0, 0.3, 0.0]
RIGHT_TRANS_IN_WORLD = [0.0, -0.3, 0.0]

# TCP poses of the left and right robot in the shared world frame to hold an object between them,
# with the two end-effectors facing each other 20 cm apart [m][]
LEFT_HOLD_POSE = [0.5, 0.1, 0.3, 0.7071068, 0.7071068, 0.0, 0.0]
RIGHT_HOLD_POSE = [0.5, -0.1, 0.3, 0.7071068, -0.7071068, 0.0, 0.0]

# Squeezing force applied on the object [N]
SQUEEZE_FORCE = 10.0

# Height to lift the object [m]
LIFT_HEIGHT = 0.1

# Linear and angular velocity of the bimanual motion [m/s] [rad/s]
LINEAR_VEL = 0.05
ANGULAR_VEL = 0.2


def move_to_hold_poses(robot_pair):
    """
    Move both robots to the holding poses and wait for them to stop.

    """

    robot_pair.SendCartesianMotionForce([LEFT_HOLD_POSE, RIGHT_HOLD_POSE])
    time.sleep(1)
    while not robot_pair.stopped():
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
        ">>> Tutorial description <<<\nThis tutorial uses the bimanual primitive MoveTensioned to "
        "squeeze an object between the end-effectors of the two robots, then lift it up and put "
        "it back down while keeping the squeezing force.\n"
    )

    try:
        # DRDK Initialization
        # ==========================================================================================
        # Instantiate robot pair interface. The base translations are needed by the bimanual
        # primitives to know where the two robots are relative to each other
        robot_pair = flexivdrdk.RobotPair(
            [args.left_robot_sn, args.right_robot_sn],
            [LEFT_TRANS_IN_WORLD, RIGHT_TRANS_IN_WORLD],
        )

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

        # Zero Force-torque Sensors
        # =========================================================================================
        robot_pair.SwitchMode(Mode.NRT_PRIMITIVE_EXECUTION)
        # IMPORTANT: must zero force/torque sensor offset, because MoveTensioned measures the
        # squeezing force using the force/torque sensors
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

        # Hold Object
        # =========================================================================================
        # Move the two end-effectors to face each other
        logger.info("Moving to holding poses")
        robot_pair.SwitchMode(Mode.NRT_CARTESIAN_MOTION_FORCE)
        move_to_hold_poses(robot_pair)

        # Wait for the user to place an object between the two end-effectors
        logger.warning(
            "Place an object about 20 cm wide between the two end-effectors, then press Enter to "
            "squeeze it. Without an object, the two robots will keep moving towards each other"
        )
        input()

        # Bimanual Tensioned Motion
        # =========================================================================================
        # Instantiate bimanual primitives interface
        bimanual_primitives = flexivdrdk.BimanualPrimitives(robot_pair)

        # Squeeze the object, then lift it up. The left robot's TCP tracks the target pose, while
        # the right robot follows along and keeps squeezing the object. A negative tension force
        # means compression
        target_pose = robot_pair.states()[0].tcp_pose.copy()
        target_pose[2] += LIFT_HEIGHT
        logger.info(
            f"Squeezing the object with {SQUEEZE_FORCE} N and lifting it up by {LIFT_HEIGHT} m"
        )
        bimanual_primitives.MoveTensioned(
            flexivdrdk.ReferenceArm.LEFT,
            target_pose,
            -SQUEEZE_FORCE,
            LINEAR_VEL,
            ANGULAR_VEL,
        )

        # Put the object back down, this time using the right robot's TCP as reference
        target_pose = robot_pair.states()[1].tcp_pose.copy()
        target_pose[2] -= LIFT_HEIGHT
        logger.info("Putting the object back down")
        bimanual_primitives.MoveTensioned(
            flexivdrdk.ReferenceArm.RIGHT,
            target_pose,
            -SQUEEZE_FORCE,
            LINEAR_VEL,
            ANGULAR_VEL,
        )

        # Release Object
        # =========================================================================================
        # Wait for the user to hold the object before releasing it
        logger.warning("Hold the object, then press Enter to release it")
        input()

        # Move the two end-effectors back to the holding poses to release the squeezing force
        logger.info("Releasing the object")
        move_to_hold_poses(robot_pair)

        logger.info("Program finished")

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
