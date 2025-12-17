# Free Fleet

[![Nightly](https://github.com/open-rmf/free_fleet/actions/workflows/nightly.yaml/badge.svg)](https://github.com/open-rmf/free_fleet/actions/workflows/nightly.yaml)[![Unit tests](https://github.com/open-rmf/free_fleet/actions/workflows/unit-tests.yaml/badge.svg)](https://github.com/open-rmf/free_fleet/actions/workflows/unit-tests.yaml)[![Nav2 Integration tests](https://github.com/open-rmf/free_fleet/actions/workflows/nav2-integration-tests.yaml/badge.svg)](https://github.com/open-rmf/free_fleet/actions/workflows/nav2-integration-tests.yaml)[![Nav1 Integration tests](https://github.com/open-rmf/free_fleet/actions/workflows/nav1-integration-tests.yaml/badge.svg)](https://github.com/open-rmf/free_fleet/actions/workflows/nav1-integration-tests.yaml)[![codecov](https://codecov.io/github/open-rmf/free_fleet/graph/badge.svg?token=JCOB9g3YTn)](https://codecov.io/github/open-rmf/free_fleet)

- **[Introduction](#introduction)**
- **[Dependency installation, source build and setup](#dependency-installation-source-build-and-setup)**
- **[Simulation examples](#simulation-examples)**
  - [Nav2 Single turtlebot3 world](#nav2-single-turtlebot3-world)
  - [Nav2 Multiple turtlebot3 world](#nav2-multiple-turtlebot3-world)
  - [Nav1 Single turtlebot3 world](#nav1-single-turtlebot3-world)
- **[Troubleshooting](#troubleshooting)**
- **[Contributing](#contributing)**
- **[TODOs](#todos)**

## Introduction

Free fleet is a python implementation of the Open-RMF Fleet Adapter, based on the [`fleet_adapter_template`](https://github.com/open-rmf/fleet_adapter_template).

This repository has been updated to use **pure ROS 2 communication** between the fleet adapter and robots (ROS 2 topics/actions/services), without any additional middleware layer.

Supports
* [Ubuntu 24.04](https://ubuntu.com/blog/ubuntu-desktop-24-04-noble-numbat-deep-dive)
* [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html)
* [rmw-cyclonedds-cpp](https://github.com/ros2/rmw_cyclonedds)
* [Open-RMF binaries on ROS 2 Jazzy](https://github.com/open-rmf/rmf)

Most of the tests have been performed using `rmw-cyclonedds-cpp`, while other RMW implementations have shown varying results. Support and testing with other RMW implementations will be set up down the road.

## Dependency installation, source build and setup

System dependencies,

```bash
sudo apt update && sudo apt install python3-pip ros-jazzy-rmw-cyclonedds-cpp
```

The dependencies `nudged`, `pycdr2`, `rosbags` are available through `pip`. Users can choose to set up a virtual environment, or `--break-system-packages` by performing the installation directly.

```bash
pip3 install nudged pycdr2 rosbags --break-system-packages
```

Set up workspace, install dependencies and build,

```bash
mkdir -p ~/ff_ws/src
cd ~/ff_ws/src
git clone https://github.com/open-rmf/free_fleet

# Install dependencies
cd ~/ff_ws
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -yr

# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Simulation examples

Examples for running a single robot or multiple robots in simulation are in `free_fleet_examples`, along with fleet configuration files for `free_fleet_adapter`.

For ROS 2, simulations will be launched using the `nav2_bringup` package. Since the `turtlebot3_gazebo` package is not being released past jazzy, users will need to clone the package to access the gazebo models,

```
sudo apt update && sudo apt install ros-jazzy-nav2-bringup

git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations ~/turtlebot3_simulations
```

### Nav2 Single turtlebot3 world

![](../media/ff_tb3_faster_smaller.gif)

This simulates running a turtlebot3 with a ROS 2 navigation stack, and setting up RMF with `free_fleet_adapter`.

Launch simulation and set up the initial position of the robot (see gif),

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_simulations/turtlebot3_gazebo/models

# Launch the simulation
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=0

# Or launch headless
# ros2 launch nav2_bringup tb3_simulation_launch.py
```

Start the RMF core packages on a different `ROS_DOMAIN_ID` to simulate running on a different machine,

```bash
source ~/ff_ws/install/setup.bash
export ROS_DOMAIN_ID=55

ros2 launch free_fleet_examples turtlebot3_world_rmf_common.launch.xml
```

Launch the `free_fleet_adapter` with the current example's configurations, verify that `nav2_tb3` has been added to fleet `turtletbot3`.

```bash
source ~/ff_ws/install/setup.bash
export ROS_DOMAIN_ID=55

ros2 launch free_fleet_examples nav2_tb3_simulation_fleet_adapter.launch.xml

# Or launch with the rmf-web API server address
# ros2 launch free_fleet_examples nav2_tb3_simulation_fleet_adapter.launch.xml  server_uri:="ws://localhost:8000/_internal"
```

Dispatch an example RMF patrol tasks using [`rmf-web`](https://github.com/open-rmf/rmf-web) on the same `ROS_DOMAIN_ID` as the RMF core packages, or use the `dispatch_patrol` script,

```bash
source ~/ff_ws/install/setup.bash
export ROS_DOMAIN_ID=55

ros2 run rmf_demos_tasks dispatch_patrol \
  -p north_west north_east south_east south_west \
  -n 2 \
  -st 0
```

Dispatch example custom actions `hello_world` and `delayed_hello_world`. These example custom actions simply prints messages, and can be observed in the terminal logs.

```bash
source ~/ff_ws/install/setup.bash
export ROS_DOMAIN_ID=55

# hello_world
ros2 run rmf_demos_tasks dispatch_action -st 0 -a hello_world

# hello_world with user name in description
ros2 run rmf_demos_tasks dispatch_action -st 0 -a hello_world \
  -ad '{"user": "John Doe"}'

# delayed_hello_world, default as 5 seconds
ros2 run rmf_demos_tasks dispatch_action -st 0 -a delayed_hello_world

# delayed_hello_world with user name and custom wait duration in description
ros2 run rmf_demos_tasks dispatch_action -st 0 -a delayed_hello_world \
  -ad '{"user": "Jane Doe", "wait_duration_sec": 20}'

# While a `delayed_hello_world` is ongoing, users can also trigger a cancellation manually on the action, which will cause the task to be cancelled as well.
ros2 topic pub --once  /cancel_delayed_hello_world std_msgs/msg/Empty "{}"
```

### Nav2 Multiple turtlebot3 world

> [!NOTE]
> This multi-robot simulation example is only for testing purposes.

![](../media/multirobot_sim_architecture.jpg)

In this example, both robots run within the same ROS 2 system and can be controlled by `free_fleet_adapter`.

![](../media/ff_unique_faster_smaller.gif)

Launch simulation, start the robots, and set up the initial positions (see gif),

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_simulations/turtlebot3_gazebo/models

ros2 launch nav2_bringup unique_multi_tb3_simulation_launch.py
```

Start the RMF core packages on a different `ROS_DOMAIN_ID` to simulate running on a different machine,

```bash
source ~/ff_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=55

ros2 launch free_fleet_examples turtlebot3_world_rmf_common.launch.xml
```

Launch the `free_fleet_adapter` with the current example's configurations, verify that `nav2_tb3` has been added to fleet `turtlebot3`.

```bash
source ~/ff_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=55

ros2 launch free_fleet_examples nav2_unique_multi_tb3_simulation_fleet_adapter.launch.xml

# Or launch with the rmf-web API server address
# ros2 launch free_fleet_examples nav2_unique_multi_tb3_simulation_fleet_adapter.launch.xml  server_uri:="ws://localhost:8000/_internal"
```

Dispatch example RMF patrol tasks using [`rmf-web`](https://github.com/open-rmf/rmf-web) on the same `ROS_DOMAIN_ID` as the RMF core packages, or use the `dispatch_patrol` scripts, which will cause the robot to negotiate as they perform their tasks.

```bash
source ~/ff_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=55

# robot1 to run clockwise around the map
ros2 run rmf_demos_tasks dispatch_patrol \
  -p north_west north_east south_east south_west \
  -n 3 \
  -st 0 \
  -F turtlebot3 \
  -R robot1

# robot2 to run anti-clockwise around the map
ros2 run rmf_demos_tasks dispatch_patrol \
  -p south_west south_east north_east north_west \
  -n 3 \
  -st 0 \
  -F turtlebot3 \
  -R robot2
```

## Troubleshooting

Check out the [Troubleshooting wiki](https://github.com/open-rmf/free_fleet/wiki/Troubleshooting).

## Contributing

* Contributions will follow guidelines from the [OSRA Open-RMF project charter](https://osralliance.org/wp-content/uploads/2024/03/open-rmf-project-charter.pdf), so for community contributions, the best way would be to fork and start a pull requests.
* Make sure commits are signed off (`--signoff`) and [GPG signed (`-S`)](https://docs.github.com/en/authentication/managing-commit-signature-verification/about-commit-signature-verification). To retroactively sign past commits on a PR, check out this [post](https://superuser.com/questions/397149/can-you-gpg-sign-old-commits).
* To test the integration tests locally, build with `colcon build --cmake-args -DNAV2_INTEGRATION_TESTING=ON`, pull docker images mentioned [here](.github/docker/integration-tests/nav2-docker-compose.yaml), start the docker compose `docker compose -f nav2-docker-compose.yaml up -d`, and test as usual using `colcon test`. This applies to the flag `-DNAV1_INTEGRATION_TESTING=ON` as well. Shut down the docker compose with `docker compose -f nav2-docker-compose.yaml down`.
* Check out [Open-RMF project board](https://github.com/orgs/open-rmf/projects/10/views/1) for tickets or ways to contribute to other projects in the Open-RMF ecosystem.
* Join the bi-weekly [Open-RMF Project Management Committee Open Sessions](https://discourse.ros.org/t/launch-of-open-rmf-project-management-committee-open-sessions/38552).

## TODOs

* attempt to optimize tf messages (not all are needed)
* map switching support
* test replanning behavior
* support for Rolling
* docker images
* releases
* testing and support for other RMW implementations
