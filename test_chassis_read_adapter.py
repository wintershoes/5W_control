#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Smoke test for the read-only chassis adapter."""

import pprint

import rospy

from chassis_read_adapter import ChassisReadAdapter


def main():
    rospy.init_node("test_chassis_read_adapter", anonymous=True)

    chassis = ChassisReadAdapter(wait_timeout=2.0)

    print("current_pose:")
    pprint.pprint(chassis.get_current_pose())

    print("\nrobot_state:")
    pprint.pprint(chassis.get_robot_state_dict())

    print("\nnav_source_used:")
    pprint.pprint(chassis.get_nav_source_used())

    print("\ncheck_chassis_ready:")
    pprint.pprint(chassis.check_chassis_ready())


if __name__ == "__main__":
    main()

