#!/usr/bin/env python

"""basics9_update_robot_tool.py

This tutorial shows how to online update and interact with the tools of both robots. All changes
made to the robot tool system will take effect immediately without needing to reboot. However, the
robots must be put into IDLE mode when making these changes.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import logging
import flexivdrdk
from flexivrdk import Mode, Tool, ToolParams  # for type definition


def print_tool_list(tool):
    """
    Print all configured tools currently in a robot's tools pool.

    """

    tool_list = tool.list()
    for i in range(len(tool_list)):
        print(f"[{i}] {tool_list[i]}")
    print()


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
        ">>> Tutorial description <<<\nThis tutorial shows how to online update and interact with "
        "the tools of both robots. All changes made to the robot tool system will take effect "
        "immediately without needing to reboot. However, the robots must be put into IDLE mode "
        "when making these changes.\n"
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

        # Update Robot Tools
        # ==========================================================================================
        # Make sure both robots are in IDLE mode
        robot_pair.SwitchMode(Mode.IDLE)

        # Instantiate tool interfaces. DRDK does not provide a tool interface for the robot pair,
        # so a flexivrdk.Tool interface is instantiated for each of the underlying flexivrdk.Robot
        # instances. The same operations are done on both robots, so put them in a dictionary
        left_robot, right_robot = robot_pair.instances()
        tools = {"left": Tool(left_robot), "right": Tool(right_robot)}

        # Get and print a list of already configured tools currently in each robot's tools pool
        for robot_name, tool in tools.items():
            logger.info(f"All configured tools of the {robot_name} robot:")
            print_tool_list(tool)

        # Get and print the current active tool of each robot
        for robot_name, tool in tools.items():
            logger.info(
                f"Current active tool of the {robot_name} robot: [{tool.name()}]"
            )

        # Set name and parameters for a new tool
        new_tool_name = "ExampleTool1"
        new_tool_params = ToolParams()
        new_tool_params.mass = 0.9
        new_tool_params.CoM = [0.0, 0.0, 0.057]
        new_tool_params.inertia = [2.768e-03, 3.149e-03, 5.64e-04, 0.0, 0.0, 0.0]
        new_tool_params.tcp_location = [
            0.0,
            -0.207,
            0.09,
            0.7071068,
            0.7071068,
            0.0,
            0.0,
        ]

        # If there's already a tool with the same name in a robot's tools pool, then remove it
        # first, because duplicate tool names are not allowed
        for robot_name, tool in tools.items():
            if tool.exist(new_tool_name):
                logger.warning(
                    f"Tool with the same name [{new_tool_name}] already exists in the "
                    f"{robot_name} robot, removing it now"
                )
                # Switch to other tool or no tool (Flange) before removing the current tool
                tool.Switch("Flange")
                tool.Remove(new_tool_name)

        # Add the new tool to both robots
        for robot_name, tool in tools.items():
            logger.info(f"Adding new tool [{new_tool_name}] to the {robot_name} robot")
            tool.Add(new_tool_name, new_tool_params)

        # Get and print the tools lists again, the new tool should appear at the end
        for robot_name, tool in tools.items():
            logger.info(f"All configured tools of the {robot_name} robot:")
            print_tool_list(tool)

        # Switch both robots to the newly added tool, i.e. set it as the active tool
        for robot_name, tool in tools.items():
            logger.info(f"Switching the {robot_name} robot to tool [{new_tool_name}]")
            tool.Switch(new_tool_name)

        # Get and print the current active tools again, should be the new tool
        for robot_name, tool in tools.items():
            logger.info(
                f"Current active tool of the {robot_name} robot: [{tool.name()}]"
            )

        # Switch to other tool or no tool (Flange) before removing the current tool
        for tool in tools.values():
            tool.Switch("Flange")

        # Clean up by removing the new tool from both robots
        time.sleep(2)
        for robot_name, tool in tools.items():
            logger.info(f"Removing tool [{new_tool_name}] from the {robot_name} robot")
            tool.Remove(new_tool_name)

        logger.info("Program finished")

    except Exception as e:
        logger.error(str(e))


if __name__ == "__main__":
    main()
