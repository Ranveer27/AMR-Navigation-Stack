#!/usr/bin/env python3
"""
go_to_goal.py - single-file "2D Nav Goal" for Nav2 + Gazebo.

Needs the simulation and Nav2 already running (sim_launch.xml + nav2_launch.xml).
Run it after sourcing ROS 2 and your workspace; no build step is needed.

  python3 go_to_goal.py 11 4 90          # go to world x=11 y=4, facing 90 degrees
  python3 go_to_goal.py 11 4             # yaw defaults to 0
  python3 go_to_goal.py                  # interactive: type "x y [yaw_deg]" again and again, q to quit
  python3 go_to_goal.py 15 2 0 --map-coords   # coordinates are already in the map frame
  python3 go_to_goal.py --init-pose      # also publish the initial pose before navigating

x, y are Gazebo WORLD coordinates (what you see in Gazebo); yaw is in DEGREES, 0 = facing +X.
"""
import argparse
import math
import sys

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

# Gazebo world pose where the robot was spawned when the SLAM map was built.
# slam_toolbox puts the map origin at that pose, so map = world relative to it.
# Change this if you mapped from a different spawn pose.
MAP_ORIGIN_IN_WORLD = (-4.0, 2.0, 0.0)   # x, y, yaw(rad)


def world_to_map(x, y, yaw):
    ox, oy, oyaw = MAP_ORIGIN_IN_WORLD
    dx, dy = x - ox, y - oy
    c, s = math.cos(oyaw), math.sin(oyaw)
    return c * dx + s * dy, -s * dx + c * dy, yaw - oyaw


class GoalSender(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self._last_print = 0.0

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header = self.make_pose(x, y, yaw).header
        msg.pose.pose = self.make_pose(x, y, yaw).pose
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        while self.init_pub.get_subscription_count() == 0 and rclpy.ok():
            self.get_logger().info('Waiting for AMCL to subscribe to /initialpose ...')
            rclpy.spin_once(self, timeout_sec=1.0)
        self.init_pub.publish(msg)
        self.get_logger().info(f'Initial pose published: map ({x:.2f}, {y:.2f}, {yaw:.2f} rad)')

    def _feedback(self, fb_msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_print > 1.0:
            self._last_print = now
            fb = fb_msg.feedback
            self.get_logger().info(f'distance remaining: {fb.distance_remaining:.2f} m')

    def go(self, x, y, yaw):
        """Send one goal (map frame) and block until it finishes. Returns True on success."""
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose action server not available. Is Nav2 running?')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(x, y, yaw)

        send_future = self.client.send_goal_async(goal, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle or not handle.accepted:
            self.get_logger().error('Goal rejected (point inside an obstacle or outside the map?)')
            return False

        result_future = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result_future)
        except KeyboardInterrupt:
            self.get_logger().warn('Ctrl-C: canceling goal')
            rclpy.spin_until_future_complete(self, handle.cancel_goal_async())
            return False

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached.')
            return True
        names = {GoalStatus.STATUS_ABORTED: 'ABORTED (no path or robot stuck)',
                 GoalStatus.STATUS_CANCELED: 'CANCELED'}
        self.get_logger().error(f'Goal {names.get(status, f"finished with status {status}")}')
        return False


def parse_goal(values):
    """values: [x, y] or [x, y, yaw_deg]  ->  (x, y, yaw_rad)"""
    if len(values) not in (2, 3):
        raise ValueError('need x y [yaw_deg]')
    x, y = float(values[0]), float(values[1])
    yaw = math.radians(float(values[2])) if len(values) == 3 else 0.0
    return x, y, yaw


def main():
    ap = argparse.ArgumentParser(description='Send a Nav2 goal like RViz 2D Nav Goal')
    ap.add_argument('goal', nargs='*', help='x y [yaw_deg]  (world coordinates). Omit for interactive mode.')
    ap.add_argument('--map-coords', action='store_true', help='goal is in map-frame coordinates')
    ap.add_argument('--init-pose', action='store_true',
                    help='publish the initial pose (spawn point) to AMCL first')
    args, _ = ap.parse_known_args()

    # simulation publishes /clock, so this node must use simulated time
    rclpy.init(args=[sys.argv[0], '--ros-args', '-p', 'use_sim_time:=true'])
    node = GoalSender()

    if args.init_pose:
        node.publish_initial_pose(0.0, 0.0, 0.0)   # spawn point = map origin

    def to_map(x, y, yaw):
        return (x, y, yaw) if args.map_coords else world_to_map(x, y, yaw)

    try:
        if args.goal:
            mx, my, myaw = to_map(*parse_goal(args.goal))
            node.get_logger().info(f'Goal in map frame: ({mx:.2f}, {my:.2f}, {myaw:.2f} rad)')
            ok = node.go(mx, my, myaw)
            code = 0 if ok else 1
        else:
            code = 0
            print('Interactive mode. Type: x y [yaw_deg]   (world coordinates)   or  q  to quit')
            while rclpy.ok():
                try:
                    line = input('goal> ').strip()
                except EOFError:
                    break
                if line.lower() in ('q', 'quit', 'exit'):
                    break
                if not line:
                    continue
                try:
                    mx, my, myaw = to_map(*parse_goal(line.replace(',', ' ').split()))
                except ValueError as e:
                    print(f'  bad input ({e}). Example: 11 4 90')
                    continue
                node.go(mx, my, myaw)
    except KeyboardInterrupt:
        code = 130
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
