# Aruco detection and angular visual servoing in Gazebo & ROS 2

**Assignment 1 — Experimental Robotics Lab**  
Authors: Gian Marco Balia, Christian Negri Ravera, Francesca Amato, Filippo Salterini, Arian Tavousi, Milad Rabiei

**Short description:**  
Spawn a robot in a Gazebo world with 5 ArUco markers placed in a circle. The system detects all markers, then, in ascending order of marker ID, rotates the robot to center each marker in the image (visual servoing phase), publishes an annotated image (with a circle around the marker) on a topic and saves the final frames. 

<table>
  <tr>
    <td>Gazebo Run</td>
     <td>Rviz panel</td>
  </tr>
  <tr>
    <td><img src="images/gazebo.gif" width=533 height=300></td>
    <td><img src="images/rviz.png" width=533 height=300></td>
  </tr>
 </table>

---

## Description and features
- Gazebo simulation with a robot and 5 ArUco-marked boxes placed in a circular arrangement.
- ArUco detection pipeline (via `ros_aruco_opencv` package).
- `aruco_detections.py` node:
  - searches for all 5 marker IDs are detected and records their transformations w.r.t the world,
  - sorts IDs and selects the lowest remaining ID as target,
  - rotates the robot to center the marker in the camera image (angular visual servoing),
  - when centered, draws a circle on the image, publishes it on `/final_marker_image` and saves to disk,
  - repeats for each marker in ascending order until done.


### Workflow and Operational Logic

The node implements a two‑stage behavior: **global detection** followed
by **marker‑by‑marker centering**.

#### 1. Detection Phase

-   The node subscribes to `/aruco_detections` and continuously rotates
    the robot by publishing a fixed angular velocity.
-   Each time an ArUco marker is detected, its transform relative to the
    robot's base frame is retrieved from TF.
-   Marker IDs are recorded until **five unique IDs** have been
    observed.
-   Once five markers are collected, the node sorts their IDs and
    switches to the **centering phase**, starting with the first marker.

#### 2. Centering Phase (Global Alignment)

-   The node selects the current target marker (e.g., `marker_12`) and
    looks up its transform relative to the robot base.
-   It computes the angular error using `atan2(y, x)` in the robot
    frame.
-   A proportional controller applies angular velocity to rotate the
    robot until the marker is approximately centered.
-   When the marker becomes visible in the image stream, the node
    switches to **local optimization**.

#### 3. Local Optimization (Visual Servoing)

-   The node uses an image‑derived variable (`local_z`) to refine the
    centering with a more direct visual cue.
-   When `|local_z|` falls below a threshold, the marker is considered
    centered precisely.

#### 4. Image Capture

-   Upon successful centering, the node:
    -   Retrieves the last received image,
    -   Draws a circle around the image center,
    -   Saves the processed image to the workspace resources directory,
    -   Publishes the modified image on `/final_marker_image`.

#### 5. Progression Through All Markers

-   After finishing one marker, the node proceeds to the next ID in the
    sorted list.
-   When all markers have been centered and captured, the node enters
    the `done` state and stops issuing velocity commands.


---

## Prerequisites

OS: Ubuntu (tested on Ubuntu 22 for ROS 2 Humble and Ubuntu 24 for ROS 2 Jazzy).

ROS 2: Humble (works) and Jazzy (works) — you may need to clone the correct branch of ros_aruco_opencv for your distro.

Gazebo Harmonic is used — confirm the exact Gazebo version matching the ROS 2 distro (although Harmonic works on both ROS versions in this assignment).

Python: system Python3 (version used by ROS 2 distro; typically 3.10+).

System tools: colcon, vcstool (python3-vcstool), development libraries for ROS2 packages.

## Installation (bundle workspace)

This repository is provided as a bundle (multiple packages + repos files). The recommended workflow is to create a workspace and use vcs to import the referenced repositories.

Clone the assignment bundle (example)

```bash
git clone https://github.com/ExpRobLab/assignment1_bundle.git assignment1_ws
cd assignment1_ws
```

Import repositories (if the bundle supplies .repos files inside the cloned repo)
from within your workspace:

```bash
vcs import src < assignment1_https.repos
```

or with SSH:

```bash
vcs import src < assignment1_ssh.repos
```

If you want to use also the simulation of the Husarion Rosbot:

```bash
sudo apt-get update
sudo apt-get install -y python3-pip ros-dev-tools

export HUSARION_ROS_BUILD_TYPE=simulation
vcs import src < src/rosbot_ros/rosbot/rosbot_${HUSARION_ROS_BUILD_TYPE}.repos

export PIP_BREAK_SYSTEM_PACKAGES=1
sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install colcon build --symlink-install --packages-up-to
-i --from-path src --rosdistro $ROS_DISTRO -y

colcon build --symlink-install --packages-up-to rosbot --cmake-args -DCMAKE_BUILD_TYPE=Release
```

build

```bash
colcon build --symlink-install --packages-up-to assignment1 bme_gazebo_basics worlds_manager aruco_opencv_msgs aruco_opencv

source install/local_setup.bash
```

## Launchfiles

There are several choices to launch:

1. MoGi bot with 2 wheels with differential driver control:

```bash
ros2 launch assignment1 assignment_2wheels.launch.py
```
2. MoGi bot with 4 wheels with skid steer control:

```bash
ros2 launch assignment1 assignment_4wheels_skid_steer.launch.py
```
3. MoGi bot with 4 wheels with mecanum drive controller:

```bash
ros2 launch assignment1 assignment_4wheels_mecanum.launch.py
```

4. Husarion Rosbot with 4 wheels with mecanum drive controller:

```bash
ros2 launch assignment1 assignment_husarion_sim.launch.py
```

5. Real Husarion Rosbot:

```bash
export ROS_DOMAIN_ID=<id_rosbot>
ros2 launch assignment1 assignment_husarion.launch.py
```

---
---






NOTE: The repo references ros_aruco_opencv external package. Make sure that package is available in your src and that you check-out (or initially clone in vs) a branch compatible with your ROS 2 distro if necessary.

External dependencies (explicit):

The bundle depends on the ArUco OpenCV ROS package:

https://github.com/fictionlab/ros_aruco_opencv.git

If you import via .repos this will be pulled automatically the Jazzy version. If not, clone it into src/.

Important: the package maintainer may have a branch per ROS distro. If using Humble or Jazzy, check out the matching branch (or the aruco_detection branch referenced by your notes).

Install apt dependencies commonly required (replace <distro> where necessary; example for Humble):

```bash
sudo apt install ros-humble-ros-base ros-humble-cv-bridge ros-humble-image-transport                  ros-humble-gazebo-ros-pkgs python3-opencv
```

## Launch files 

- `assignment1/launch/assignment.launch.py` — recommended main demo launch (spawns robot, relevant nodes and world).

## Nodes, topics & frames 

### Important nodes
- **aruco_detection_node** (script: `aruco_detections.py`) — core assignment logic.
- **aruco_tracker** (from `ros_aruco_opencv`) — publishes detections to `/aruco_detections`.

### Topics (publish / subscribe)

Subscribed:

- `/aruco_detections`
- `/camera/image/compressed`

Published:

- `/cmd_vel`
- `/final_marker_image`

### TF frames used

- `odom`
- `base_footprint`
- `marker_<ID>`

## Detailed explanation of aruco_detections.py logic

### Node lifecycle & subscriptions

Creates TF buffer, subscribes to detection and image topics, publishes velocity and images, uses a control loop.

### Detection Phase (and callback)

- Rotates robot while detecting
- For each detected marker:
  - TF lookup `odom -> marker_<id>`
  - Records transform + ID
- When all 5 detected:
  - Sorts IDs, selects lowest, enters "centering" state

### Control loop

- Active only in "centering"
- TF lookup: `base_footprint -> marker_<id>`
- Compute angle using atan2
- P-controller for angular velocity
- When centered:
  - publish stop
  - save + publish annotated image
  - state = "done"

### Image handling & annotation

- Converts compressed → OpenCV
- Draws circle on marker center
- Saves PNG
- Publishes annotated image

## Parameters & tuning

- `Kp = 1.0`
- `max_angular = 1.0`
- `threshold = 0.1 rad`
- `timer: 0.01s`

## Output files, RQt graphs & screenshots

- Output Images
<table>
  <tr>
    <td>Box 1</td>
    <td>Box 2</td>
    <td>Box 3</td>
  </tr>
  <tr>
    <td><img src="images/box1.png" width=533 height=300></td>
    <td><img src="images/box2.png" width=533 height=300></td>
    <td><img src="images/box3.png" width=533 height=300></td>
  </tr>
 </table>

- RQT Graph
<table>
  <tr>
    <td><img src="images/rqt.png" width=533 height=300></td>
  </tr>
 </table>

## Troubleshooting & tips

Camera topic mismatch:

```bash
ros2 topic list | grep image
```

## CLI snippets

```bash
ros2 node list
ros2 topic echo /aruco_detections
ros2 topic hz /camera/image/compressed
```
