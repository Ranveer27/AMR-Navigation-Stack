A differential-drive Autonomous Mobile Robot (AMR) built with ROS 2
Humble and simulated in a custom warehouse environment using Gazebo
Fortress. The project uses **ros2_control** for wheel control and
encoder-based odometry, **SLAM Toolbox** for mapping, and **Nav2** for
autonomous navigation.

**Stack:** ROS 2 Humble · Gazebo Fortress (Ignition) · ros_gz ·
ros2_control · slam_toolbox · Nav2 · Python

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Fortress-orange)
![Navigation](https://img.shields.io/badge/Navigation-Nav2-blue)
![License](https://img.shields.io/badge/License-To%20be%20specified-lightgrey)

> **Project status:** Simulation and navigation development. Add a demo
> GIF or screenshot below when available.

## Table of Contents

-   [Features](#1-features)
-   [System Architecture](#2-system-architecture)
-   [Requirements](#3-requirements)
-   [Package Structure](#4-package-structure)
-   [Quick Start](#5-quick-start)
-   [How It Works](#6-how-it-works)
-   [File Reference](#7-file-reference)
-   [FAQ](#8-faq-understanding-the-concepts)
-   [Problems Solved](#9-problems-solved-while-building-this)
-   [Troubleshooting](#10-troubleshooting)
-   [Future Work](#11-future-work)
-   [License and Author](#12-license-and-author)




https://github.com/user-attachments/assets/7e673546-9318-4f01-9537-f1b495a50aa1
------------------------------------------------------------------------

<img width="1918" height="1055" alt="Screenshot from 2026-10-10 12-42-10" src="https://github.com/user-attachments/assets/e7424b55-f14c-49a1-95e4-27312ac32298" />


## 1. Features

-   Robot model in URDF/Xacro: chassis, two drive wheels, front caster,
    2D LiDAR, camera
-   Custom warehouse world (four rooms joined by openings, shelves and
    obstacles)
-   `ros2_control` with `diff_drive_controller` and
    `joint_state_broadcaster` (Gazebo hardware interface)
-   Wheel-encoder odometry published as `/diff_drive_controller/odom`
    and the `odom -> base_footprint` TF
-   SLAM mapping with `slam_toolbox` (scan matching and loop closure),
    map saved for later use
-   Navigation with Nav2: AMCL localization, global and local costmaps,
    NavFn planner, DWB controller
-   Python tools to send goals by coordinates (like RViz "Nav2 Goal")
    and to patrol waypoints
-   XML launch files for the full pipeline

## 2. System architecture

    Gazebo (warehouse world + robot)
      |- lidar   --/scan---------> ros_gz_bridge --> slam_toolbox / AMCL / costmaps
      |- camera  --/camera/image-> ros_gz_bridge --> RViz
      '- wheel joints <--> ign_ros2_control <--> controller_manager
                                                  |- joint_state_broadcaster --> /joint_states --> robot_state_publisher --> TF
                                                  '- diff_drive_controller ----> /diff_drive_controller/odom
                                                                                 + TF odom -> base_footprint
                                                              ^
    /cmd_vel (teleop or Nav2) --> cmd_vel_relay --> /diff_drive_controller/cmd_vel_unstamped

**TF tree at runtime** (and who publishes each link):

    map --> odom --> base_footprint --> chassis_link --> left_wheel, right_wheel, castor_wheel
     ^        ^            ^                          |-> laser_frame
     |        |            |                          '-> camera_link --> camera_optical_link
     |        |            '-- robot_state_publisher (from the URDF and /joint_states)
     |        '-- diff_drive_controller (wheel odometry)
     '-- slam_toolbox while mapping, AMCL while navigating

## 3. Requirements

-   Ubuntu 22.04, ROS 2 Humble
-   Gazebo Fortress (Ignition Gazebo 6)
-   A GPU or working OpenGL (the lidar sensor uses the `ogre2` renderer)

``` bash
sudo apt install ros-humble-ros-gz ros-humble-ign-ros2-control ros-humble-ros2-controllers \
  ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard ros-humble-xacro ros-humble-rviz2
```

## 4. Package structure

    automatic_mobile_robot/
    |- config/
    |  |- controllers.yaml        ros2_control: controller manager, diff drive, odometry
    |  |- gazebo_bridge.yaml      Gazebo <-> ROS topic bridge
    |  |- Slam_param.yaml         slam_toolbox parameters
    |  '- nav2_params.yaml        Nav2 parameters (AMCL, costmaps, planner, controller)
    |- description/               robot model (xacro)
    |  |- my_robot.xacro          top-level, includes the rest
    |  |- materials.xacro         colours
    |  |- robot_make.xacro        chassis, wheels, caster
    |  |- laser_frame.xacro       lidar mount
    |  |- camera_link.xacro       camera mount
    |  |- robot_gazebo.xacro      Gazebo friction, lidar and camera sensors
    |  '- robot_ros2_control.xacro  ros2_control interfaces + Gazebo plugin
    |- launch/
    |  |- sim_slam_launch.xml     simulation + SLAM + RViz (make a map)
    |  |- sim_launch.xml          simulation only
    |  |- nav2_launch.xml         Nav2 + RViz (needs a running simulation)
    |  '- navigation_launch.xml   simulation, then Nav2 after a delay (one command)
    |- scripts/
    |  |- go_to_goal.py           send a goal by coordinates (single or interactive)
    |  |- navigate.py             patrol waypoints
    |  '- cmd_vel_relay.py        /cmd_vel -> diff_drive_controller input
    |- worlds/warehouse_world.sdf
    |- rviz/view_robot.rviz
    |- CMakeLists.txt
    '- package.xml

## 5. Quick start

### Build

``` bash
cd ~/ros2_workspace
colcon build --packages-select automatic_mobile_robot --symlink-install
source install/setup.bash
```

### Clean up old runs (do this before every relaunch)

A Gazebo process that survived a previous Ctrl-C will cause "controller
already loaded" errors.

``` bash
pkill -9 -f "ign gazebo"; pkill -9 -f ruby; pkill -9 -f parameter_bridge
pkill -9 -f robot_state_publisher; pkill -9 -f rviz2
ros2 daemon stop; ros2 daemon start
```

### A) Build a map with SLAM

``` bash
# Terminal 1
ros2 launch automatic_mobile_robot sim_slam_launch.xml

# Terminal 2: drive slowly and make loops so loop closure can correct drift
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3: save the map when everything is covered
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/warehouse_map --ros-args -p use_sim_time:=true
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/<user>/maps/warehouse_posegraph'}"
```

Start the mapping run from the same spawn pose you will use for
navigation (see FAQ 3).

### B) Navigate in the saved map

``` bash
# Terminal 1: simulation, then Nav2 + RViz about 30 s later
ros2 launch automatic_mobile_robot navigation_launch.xml
#   options: map:=/path/to/map.yaml   nav_delay:=45.0

# Terminal 2: send goals
ros2 run automatic_mobile_robot go_to_goal.py 11 4 90    # world x y yaw_degrees
ros2 run automatic_mobile_robot go_to_goal.py            # interactive mode
ros2 run automatic_mobile_robot navigate.py              # patrol all four rooms
```

You can also click **Nav2 Goal** in RViz and drag on the map.

### Health checks

``` bash
ros2 control list_controllers        # joint_state_broadcaster and diff_drive_controller: active
ros2 action list                     # contains /navigate_to_pose
ros2 run tf2_ros tf2_echo map odom   # prints a transform
ros2 run tf2_tools view_frames       # draws the TF tree to a PDF
```

------------------------------------------------------------------------

## 6. How it works

### 6.1 Frames and TF (read this first)

A **frame** is just the name of a coordinate system. **TF** is the ROS
system that keeps track of how frames are positioned relative to each
other, as a tree where every frame has exactly one parent.

There are two kinds of frames:

  -----------------------------------------------------------------------
  Kind                    Examples                Described by
  ----------------------- ----------------------- -----------------------
  Parts of the robot body `base_footprint`,       the **URDF** (links and
                          `chassis_link`,         joints)
                          `left_wheel`,           
                          `laser_frame`,          
                          `camera_link`           

  Reference frames of the `map`, `odom`           **not** the URDF;
  world                                           created at runtime by
                                                  the nodes that publish
                                                  them
  -----------------------------------------------------------------------

The URDF describes a physical object. `map` and `odom` are not parts of
the robot. They are coordinate systems that say *where the robot is*.
The TF tree is built from messages on the `/tf` and `/tf_static` topics,
not from the URDF. Any frame exists as soon as some node publishes a
transform that mentions it.

**Why the `map` and `odom` links were removed from the URDF.** The
original robot file declared `map` and `odom` as links and joined them
with fixed joints: `map -> odom -> base_footprint`. This caused three
problems:

1.  `robot_state_publisher` publishes fixed joints as *static*
    transforms that never change. So TF was told permanently "the robot
    is exactly at the odom origin, and odom is exactly at the map
    origin".
2.  The controller also publishes `odom -> base_footprint`, and SLAM
    publishes `map -> odom`. A frame can have only one parent, so two
    sources were publishing the same transform with different values.
    The result is a robot that flickers between positions or never
    appears to move.
3.  Gazebo converts the URDF into a physical model, so `map` and `odom`
    would become links of the robot.

The fix is to leave them out. The URDF root is `base_footprint`, and the
other nodes supply the frames above it.

**How odometry works without an `odom` frame in the URDF.** In
`controllers.yaml`:

``` yaml
odom_frame_id: odom
base_frame_id: base_footprint
enable_odom_tf: true
```

This tells `diff_drive_controller` to publish a transform with parent
`odom` and child `base_footprint`. The first time it does, TF creates
the `odom` frame. The sequence at start-up is:

1.  The controller starts and sets `odom -> base_footprint` to (0, 0,
    0). The robot's starting point is the origin of `odom`.
2.  As the wheels turn, the controller updates that transform from the
    wheel encoder readings.
3.  `robot_state_publisher` publishes everything below `base_footprint`
    (chassis, wheels, lidar, camera) from the URDF.
4.  `slam_toolbox` (mapping) or AMCL (navigation) publishes
    `map -> odom`.

You can check this yourself:

``` bash
ros2 run tf2_ros tf2_echo odom base_footprint     # changes as the robot moves
ros2 run tf2_ros tf2_echo map odom                # the drift correction
ros2 topic echo /tf_static --once                 # only the fixed URDF joints appear here
```

### 6.2 ros2_control

ros2_control separates **what you want the robot to do** from **how the
hardware does it**.

-   **Hardware interface.** The layer that talks to motors and encoders.
    In simulation it is `ign_ros2_control/IgnitionSystem`, which sets
    wheel velocities in Gazebo and reads wheel positions back. On a real
    robot you would replace only this plugin with a motor-driver plugin.
-   **Interfaces.** Each wheel joint offers a *command* interface
    (`velocity`) and *state* interfaces (`position`, `velocity`). These
    are declared in `robot_ros2_control.xacro`.
-   **Controller manager.** Loads and runs controllers in a loop (100
    Hz, `update_rate`). It runs inside Gazebo through the
    `ign_ros2_control-system` plugin.
-   **Controllers.**
    -   `diff_drive_controller` converts a velocity command (linear *v*,
        angular *w*) into wheel speeds and computes odometry:
        `w_left = (v - w*W/2) / r`, `w_right = (v + w*W/2) / r`, with W
        = 0.35 m (wheel separation) and r = 0.08 m (wheel radius).
    -   `joint_state_broadcaster` publishes wheel angles on
        `/joint_states` so the wheels move in TF.
-   **Spawners.** The `controller_manager spawner` commands in the
    launch files load and activate the controllers.

### 6.3 Odometry

Odometry estimates position by counting wheel rotation (dead reckoning).
Each control cycle:

    ds     = r * (dphi_left + dphi_right) / 2       distance travelled
    dtheta = r * (dphi_right - dphi_left) / W       heading change
    x     += ds * cos(theta + dtheta/2)
    y     += ds * sin(theta + dtheta/2)
    theta += dtheta

`dphi` is the change in wheel angle (the encoder reading). With
`position_feedback: true` the controller uses these angles. The result
is published as `nav_msgs/Odometry` and as the `odom -> base_footprint`
TF.

Odometry is smooth but **drifts** (see FAQ 2). SLAM and AMCL correct the
drift through `map -> odom`.

### 6.4 SLAM (slam_toolbox)

SLAM builds a map and works out the robot's position in it at the same
time.

-   **Scan matching:** each lidar scan is aligned against recent scans,
    giving a better pose than odometry alone.
-   **Pose graph:** scans are stored as nodes, and the alignments
    between them are constraints.
-   **Loop closure:** when the robot returns to a known place, a new
    constraint is added and a solver (Ceres) adjusts the whole graph to
    remove accumulated drift.
-   **Output:** the `/map` occupancy grid and the `map -> odom`
    transform.

Saving produces `.pgm` and `.yaml` (the occupancy map, used by Nav2) and
`.posegraph` and `.data` (the full graph, used to continue mapping
later).

### 6.5 Navigation (Nav2)

  -----------------------------------------------------------------------
  Component                           Job
  ----------------------------------- -----------------------------------
  `map_server`                        Loads the saved map

  `amcl`                              Localizes the robot in the map with
                                      a particle filter; publishes
                                      `map -> odom`

  global costmap                      The whole map plus obstacles,
                                      inflated for safety

  local costmap                       A small window around the robot,
                                      from live lidar data

  `planner_server` (NavFn)            Computes a global path to the goal

  `controller_server` (DWB)           Follows the path and avoids nearby
                                      obstacles; outputs velocity
                                      commands

  `behavior_server`                   Recoveries when stuck (spin, back
                                      up, wait)

  `bt_navigator`                      Behavior tree that coordinates all
                                      of the above; offers the
                                      `NavigateToPose` action

  `waypoint_follower`                 Drives through a list of poses

  `velocity_smoother`                 Limits acceleration; publishes
                                      `/cmd_vel`

  `lifecycle_manager`                 Starts and stops the nodes in the
                                      right order
  -----------------------------------------------------------------------

------------------------------------------------------------------------

## 7. File reference

### description/ (the robot)

  -----------------------------------------------------------------------
  File                                Purpose
  ----------------------------------- -----------------------------------
  `my_robot.xacro`                    Top level; includes the other files

  `materials.xacro`                   Colour definitions

  `robot_make.xacro`                  Links and joints: `base_footprint`,
                                      chassis (0.5 x 0.3 x 0.15 m), two
                                      drive wheels, front caster, masses
                                      and inertias

  `laser_frame.xacro`                 The lidar mount link

  `camera_link.xacro`                 Camera link and its optical frame

  `robot_gazebo.xacro`                Gazebo-only settings: wheel
                                      friction, `gpu_lidar` (360 samples,
                                      10 m, topic `/scan`) and the camera
                                      sensor

  `robot_ros2_control.xacro`          `<ros2_control>` joint interfaces
                                      and the Gazebo plugin that loads
                                      the controller manager
  -----------------------------------------------------------------------

### config/ (parameters)

  -----------------------------------------------------------------------
  File                                Purpose
  ----------------------------------- -----------------------------------
  `controllers.yaml`                  Update rate, controller types,
                                      wheel names, separation and radius,
                                      odometry frames, speed and
                                      acceleration limits

  `gazebo_bridge.yaml`                Topics mirrored from Gazebo into
                                      ROS: `/clock`, `/scan`,
                                      `/camera/image`,
                                      `/camera/camera_info`

  `Slam_param.yaml`                   Frames, 0.05 m resolution, scan
                                      range, travel thresholds for adding
                                      scans, loop-closure settings

  `nav2_params.yaml`                  AMCL initial pose, robot footprint,
                                      costmaps, planner, controller,
                                      speed limits, odometry topic
  -----------------------------------------------------------------------

### launch/ (starting everything)

  -----------------------------------------------------------------------
  File                                Purpose
  ----------------------------------- -----------------------------------
  `sim_slam_launch.xml`               Mapping: robot_state_publisher,
                                      Gazebo, spawn, bridge, controllers,
                                      slam_toolbox, RViz

  `sim_launch.xml`                    Simulation only (no SLAM or RViz),
                                      plus the cmd_vel relay

  `nav2_launch.xml`                   Nav2 with the saved map, plus RViz

  `navigation_launch.xml`             Includes `sim_launch.xml`, then
                                      starts Nav2 after a timer so it
                                      does not start before the
                                      simulation is ready
  -----------------------------------------------------------------------

### scripts/

  --------------------------------------------------------------------------------
  File                                Purpose
  ----------------------------------- --------------------------------------------
  `cmd_vel_relay.py`                  Forwards `/cmd_vel` to
                                      `/diff_drive_controller/cmd_vel_unstamped`
                                      (the topic the Humble controller listens on)

  `go_to_goal.py`                     Sends a `NavigateToPose` goal at given
                                      coordinates, or runs an interactive prompt;
                                      converts world to map coordinates

  `navigate.py`                       Patrols a list of waypoints using Nav2's
                                      simple commander
  --------------------------------------------------------------------------------

### Other

-   `worlds/warehouse_world.sdf`: the warehouse plus the Gazebo physics,
    sensors and scene plugins, and lighting
-   `rviz/view_robot.rviz`: RViz layout for mapping
-   `CMakeLists.txt`, `package.xml`: installs folders and scripts,
    declares dependencies

------------------------------------------------------------------------

## 8. FAQ: understanding the concepts

### 1. Why ros2_control instead of Gazebo's built-in DiffDrive plugin?

Gazebo's DiffDrive plugin only exists in the simulator. If you used it,
your control and odometry code would have to be rewritten for a real
robot. With ros2_control, the controller configuration
(`controllers.yaml`), the odometry maths, the limits and the launch
files stay the same. Only the hardware plugin changes: `IgnitionSystem`
in simulation, a motor-driver plugin on real hardware. It also gives you
standard features: velocity and acceleration limits, command timeouts
and odometry covariances.

### 2. Where does odometry drift come from, and what corrects it?

Odometry is built by adding up small movements, so every small error is
kept and grows. Sources of error: - wheel slip on the floor - a wheel
radius or wheel separation that is slightly wrong - encoder quantization
(limited resolution) - small heading errors, which turn into large
position errors over distance

Example: you drive 10 m straight and odometry says (10.0, 0.0), but the
robot is really at (9.7, 0.2). Nothing in the wheel data can reveal
that. What corrects it is something that looks at the outside world:
lidar scan matching in SLAM, or AMCL matching scans against a known map.

### 3. What is the difference between the `odom` and `map` frames?

  -----------------------------------------------------------------------
                          `odom`                  `map`
  ----------------------- ----------------------- -----------------------
  Behaviour               Smooth and continuous,  Globally consistent,
                          never jumps             can jump when corrected

  Accuracy                Good short-term, drifts Good long-term
                          long-term               

  Source                  Wheel encoders          SLAM or AMCL

  Used for                Local control and local Goals and global
                          costmap                 planning
  -----------------------------------------------------------------------

The link between them is the transform `map -> odom`, which holds the
**correction**. Using the earlier example: odometry says (10.0, 0.0),
the true pose is (9.7, 0.2), so AMCL publishes a `map -> odom` of about
(-0.3, +0.2). Then
`map -> base_footprint = (map -> odom) * (odom -> base_footprint)` gives
(9.7, 0.2) in the map, while the `odom -> base_footprint` part stays
smooth.

The origin of the map frame is the robot's pose when mapping started. If
you spawn at world (-4, 2) when mapping, that point is map (0, 0), and
world coordinates must be converted before being sent as goals.
`go_to_goal.py` does this using `MAP_ORIGIN_IN_WORLD`.

### 4. What does loop closure do?

While driving around a loop, small errors build up. When the robot comes
back to the start, the walls it sees no longer line up with the walls it
mapped at the beginning, so the map shows doubled or smeared walls. Loop
closure detects that the robot is back at a known place by matching the
current scan against old scans nearby (`loop_search_maximum_distance`).
It adds a constraint saying "these two poses are the same place", and
the solver spreads the accumulated error over the whole path. The walls
then snap into a single clean line. It is switched on with
`do_loop_closing: true` in `Slam_param.yaml`.

### 5. What does AMCL do that SLAM doesn't?

-   **SLAM** works with an *unknown* map. It builds the map and
    localizes at once, which is heavy work.
-   **AMCL** works with a *known, fixed* map. It only localizes. It
    keeps hundreds of guesses ("particles", the green cloud in RViz) of
    where the robot might be, scores each one by how well the current
    lidar scan matches the map, then keeps the good ones. It needs an
    initial pose estimate to start.

Both publish `map -> odom`. For daily use, mapping is done once with
SLAM, and Nav2 then runs AMCL on the saved map.

### 6. Why did the lidar need a frame ID fix?

Every message has a `frame_id` that says which coordinate frame its data
is expressed in. For a lidar scan it must be the frame of the lidar.
SLAM looks up the transform from that frame to the robot and the map in
TF. Gazebo Fortress, by default, names the frame after the sensor's
internal path (something like `my_robot/laser_frame/gpu_lidar`), which
does not exist in the TF tree. SLAM cannot transform the scans, so it
silently drops them and no map appears. Setting
`<ignition_frame_id>laser_frame</ignition_frame_id>` makes the scan use
`laser_frame`, which does exist (it comes from the URDF).

------------------------------------------------------------------------

## 9. Problems solved while building this

1.  **TF conflicts:** static `map` and `odom` links in the URDF fought
    with SLAM and the controller (section 6.1).
2.  **Wheel direction:** mirrored joint axes made the robot drive
    backwards; fixed by rotating the joint frames.
3.  **Stability:** with no caster and tiny masses the robot tipped over;
    fixed with a frictionless caster and realistic masses.
4.  **Lidar frame ID:** see FAQ 6.
5.  **Gazebo version differences:** plugin names and bridge message
    types differ between Fortress and newer releases.
6.  **Bridge cleanup:** wrong message type for TF, duplicate entries,
    and a wrong world name.
7.  **Launch parameter parsing:** colons inside XML comments broke the
    robot description because ROS reads it as YAML.
8.  **Map vs world coordinates:** see FAQ 3.
9.  **Start-up order:** Nav2 hangs if it starts before the simulation is
    ready; a timer delays it.
10. **Duplicate controller managers:** a leftover Gazebo process caused
    "controller already loaded" errors.

## 10. Troubleshooting

  -------------------------------------------------------------------------------------------------
  Symptom                                          Likely cause and fix
  ------------------------------------------------ ------------------------------------------------
  `Failed to configure controller` /               Old simulation still running. Run the cleanup
  `Controller already loaded`                      commands in section 5

  `navigate_to_pose action server not available`   Nav2 is not active yet. Check
                                                   `ros2 lifecycle get /bt_navigator` and the map
                                                   path

  No `/scan`                                       Check
                                                   `ros2 topic echo /scan --once | grep frame_id`
                                                   prints `laser_frame`; confirm the Sensors plugin
                                                   is in the world

  "Message filter dropped" in SLAM                 Missing `odom -> base_footprint`: check
                                                   controllers are `active`

  Robot drives backwards                           Wheel joint axis direction in `robot_make.xacro`

  No `/camera/camera_info`                         Add the CameraInfo entry to `gazebo_bridge.yaml`

  RViz Camera panel shows a garbled overlay        Set the Camera display's *Image Rendering* to
                                                   `background only`, or use the Image display

  `ros2 run ... go_to_goal.py` says no executable  Add the script to `install(PROGRAMS ...)` in
  found                                            `CMakeLists.txt` and rebuild

  Goal rejected or aborted                         The point is inside an obstacle or outside the
                                                   map; choose open floor
  -------------------------------------------------------------------------------------------------

### Author

-   **Author:** Ranveer Arya
-   **GitHub:** https://github.com/Ranveer27
-   **Contact:** ranveerarya6200@gmail.com

