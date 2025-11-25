# Assignment 1 - Bundle

This bundle include:

<!-- TODO add the inclusions -->

## Exteral dependecies

https://husarion.com/manuals/rosbot/
https://github.com/fictionlab/ros_aruco_opencv.git

## Installation

### 1. Install *vcs*

```bash
sudo apt install python3-vcstool
```

### 2. Go to your workspace and mae the `\src` folder

```bash
cd ~/path/to/your/workspace
mkdir -p src
```

### 3. Import the repositories into `\src`

You can choose between HTTPS version or SSH version.

#### HTTPS version:

```bash
vcs import src < assignment1_https.repos
```

#### SSH version:

```bash
vcs import src < assignment1_ssh.repos
```

### Build workspace

```bash
colcon build
```
