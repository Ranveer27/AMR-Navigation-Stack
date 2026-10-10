#!/usr/bin/env python3
"""
Navigate the robot inside the saved SLAM map with Nav2 (nav2_simple_commander).

  ros2 run automatic_mobile_robot navigate.py                       # patrol the whole warehouse once
  ros2 run automatic_mobile_robot navigate.py --loops 3             # patrol 3 times
  ros2 run automatic_mobile_robot navigate.py --goal 11 4 1.57      # single goal: x y yaw(rad), Gazebo world coords
  ros2 run automatic_mobile_robot navigate.py --goal 15 2 0 --map-coords   # same, but given in map coords
  ros2 run automatic_mobile_robot navigate.py --skip-initial-pose   # do not re-publish the initial pose
"""
import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Gazebo WORLD pose where the robot was spawned when the map was built.
# slam_toolbox puts the origin of the saved map at the robot's starting pose, so a point
# with world coordinates (x, y) has map coordinates relative to this pose
# (the spawn point itself is map (0, 0, 0)).
# If you built the map from a different spawn pose, change this line.
MAP_ORIGIN_IN_WORLD = (-4.0, 2.0, 0.0)
START_POSE = (-4.0, 2.0, 0.0)  # world coordinates of the robot at launch

# (x, y, yaw) in WORLD (Gazebo) coordinates. All points are in free space and the route
# goes through the openings of the warehouse walls. The last point returns to the start.
PATROL_ROUTE = [
    (-2.5, 8.0, 0.0),     # top-left room, near the upper opening
    (3.0, 7.0, 0.0),      # through the upper opening -> top-right room
    (11.0, 4.0, -1.57),   # top-right room
    (0.0, 0.0, 0.0),      # warehouse centre
    (4.0, -3.0, 0.0),     # bottom-right room
    (11.0, -3.0, 3.14),   # bottom-right room, far side
    (3.0, -7.0, 3.14),    # through the lower opening
    (-3.0, -7.0, 3.14),   # bottom-left room
    (-11.0, -3.0, 1.57),  # bottom-left room, far side
    START_POSE,           # back to start
]


def world_to_map(x, y, yaw):
    """Convert Gazebo world coordinates to coordinates in the saved SLAM map."""
    ox, oy, oyaw = MAP_ORIGIN_IN_WORLD
    dx, dy = x - ox, y - oy
    c, s = math.cos(oyaw), math.sin(oyaw)
    return (c * dx + s * dy, -s * dx + c * dy, yaw - oyaw)


def make_pose(navigator, x, y, yaw):
    """PoseStamped in the map frame; x, y, yaw must already be map coordinates."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def wait_for_task(navigator, label):
    """Block until the current Nav2 task finishes, printing progress."""
    tick = 0
    while not navigator.isTaskComplete():
        tick += 1
        fb = navigator.getFeedback()
        if fb and tick % 20 == 0:
            if hasattr(fb, 'distance_remaining'):
                print(f'[{label}] distance remaining: {fb.distance_remaining:.2f} m')
            elif hasattr(fb, 'current_waypoint'):
                print(f'[{label}] heading to waypoint {fb.current_waypoint + 1}/{len(PATROL_ROUTE)}')
        rclpy.spin_once(navigator, timeout_sec=0.1)
    return navigator.getResult()


def report(result, label):
    if result == TaskResult.SUCCEEDED:
        print(f'[{label}] succeeded')
    elif result == TaskResult.CANCELED:
        print(f'[{label}] canceled')
    elif result == TaskResult.FAILED:
        print(f'[{label}] FAILED')
    else:
        print(f'[{label}] unknown result')
    return result == TaskResult.SUCCEEDED


def main():
    parser = argparse.ArgumentParser(description='Nav2 navigation in the saved warehouse map')
    parser.add_argument('--goal', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                        help='navigate to a single goal instead of patrolling (Gazebo world coordinates)')
    parser.add_argument('--map-coords', action='store_true',
                        help='--goal values are map-frame coordinates instead of Gazebo world coordinates')
    parser.add_argument('--loops', type=int, default=1, help='number of patrol laps (default 1)')
    parser.add_argument('--skip-initial-pose', action='store_true',
                        help='do not publish the initial pose (use if you set it in RViz)')
    args, _ = parser.parse_known_args()

    # The simulation publishes /clock, so this node must use simulated time too.
    rclpy.init(args=[sys.argv[0], '--ros-args', '-p', 'use_sim_time:=true'])
    navigator = BasicNavigator()

    if not args.skip_initial_pose:
        mx, my, myaw = world_to_map(*START_POSE)
        navigator.setInitialPose(make_pose(navigator, mx, my, myaw))
        print(f'Initial pose: world {START_POSE} -> map ({mx:.2f}, {my:.2f}, {myaw:.2f})')

    print('Waiting for Nav2 (amcl + bt_navigator) to become active...')
    navigator.waitUntilNav2Active()
    print('Nav2 is active.')

    try:
        if args.goal:
            x, y, yaw = args.goal if args.map_coords else world_to_map(*args.goal)
            print(f'Goal in map frame: ({x:.2f}, {y:.2f}, {yaw:.2f})')
            navigator.goToPose(make_pose(navigator, x, y, yaw))
            report(wait_for_task(navigator, 'goal'), 'goal')
        else:
            for lap in range(1, args.loops + 1):
                label = f'patrol {lap}/{args.loops}'
                print(f'Starting {label}')
                route = [make_pose(navigator, *world_to_map(*wp)) for wp in PATROL_ROUTE]
                navigator.followWaypoints(route)
                if not report(wait_for_task(navigator, label), label):
                    break
    except KeyboardInterrupt:
        print('Ctrl-C: canceling current task')
        navigator.cancelTask()

    navigator.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
