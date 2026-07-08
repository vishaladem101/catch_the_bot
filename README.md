# 🤖 Catch The Bot (Multi-Robot Gazebo Sim)

A dynamic and interactive ROS 2 (Robot Operating System 2) application where a master robot autonomously hunts down and "catches" target robots spawning at random locations in a native **Gazebo Sim** environment.

This project demonstrates advanced ROS 2 concepts, including **Multi-Robot Simulation**, **TF (Transform Framework) Manipulation**, and **Dynamic Process Lifecycle Management**.

---

## 🚀 Description

In this application, target robots are dynamically spawned into the Gazebo world. A master robot (`robot1`) is controlled by a closed-loop proportional controller (`noisy_boy_controller`) that calculates distance and heading errors based on ground-truth odometry to the nearest target robot.

Once the master robot gets within a catch radius (less than 0.6 meters):

1. It requests the **Spawner Node** (`noisy_boy_spawner`) to remove the target from its active tracking array via the `/catch_bot` custom service.
2. The spawner node safely terminates the target robot's dedicated background processes (State Publisher, Bridges) and removes the entity directly from the Gazebo simulation.
3. The spawner immediately spawns a new target bot, and the master robot begins tracking it.

---

## 🌟 Key Features

### 1. Multi-Robot Simulation & Namespacing

Simulating multiple robots in Gazebo requires strict isolation to prevent topic and frame collisions. The `noisy_boy_spawner` accomplishes this by dynamically namespacing every new robot (`robot1`, `robot2`, etc.):

- **Xacro Injection**: It programmatically runs `xacro` with a `namespace:=<robot_name>` argument.
- **Dynamic Subprocesses**: For every spawned robot, the spawner launches a dedicated `robot_state_publisher` and `ros_gz_bridge` in isolated Python `subprocess` process groups (`os.setsid`). This ensures that each robot has its own TF tree and bridging without manual launch file configuration.
- **Clean Teardown**: When a robot is caught, its specific process group is cleanly terminated using `SIGINT` and `SIGKILL`, avoiding zombie processes or resource leaks.

### 2. TF (Transform Framework) Handling

To coordinate the master robot and multiple target robots in a single environment, a unified TF tree is required.

- **Static TF Broadcasting**: The spawner uses a `tf2_ros.static_transform_broadcaster` to dynamically stitch each new robot's local odometry frame (`<robot_name>/odom`) to a shared global `map` frame (`map -> robotX/odom`).
- **Transform Latching**: Because static transforms are latched on the `/tf_static` topic, the spawner maintains an active list of all active robot transforms and re-broadcasts the entire array whenever a robot is spawned or caught. This prevents new static transforms from overwriting the older ones.

### 3. RViz Marker Visualization

The spawner publishes custom `visualization_msgs/MarkerArray` messages to visually represent the target bots in RViz (drawing the body, wheels, and lidar) and dynamically deletes the markers when a robot is caught.

---

## 🛠️ Tech Stack

- **Framework:** [ROS 2 (Robot Operating System 2)](https://docs.ros.org/)
- **Simulation Engine:** Gazebo Sim (Harmonic)
- **Languages:** Python (Controllers, Spawners, Launch Files)
- **Packages:** `noisy_boy_bringup`, `noisy_boy_controller`, `noisy_boy_spawner`, `noisy_boy_interfaces`, `noisy_boy_description`

---

## 📦 Installation & Usage

Ensure you have ROS 2 and Gazebo Sim installed, along with `ros_gz_bridge` and `ros_gz_sim`.

### 1. Create a Workspace and Clone

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/vishaladem101/catch_the_bot.git
cd ~/ros2_ws
```

### 2. Build the Workspace

```bash
colcon build --packages-select noisy_boy_interfaces noisy_boy_description noisy_boy_spawner noisy_boy_controller noisy_boy_bringup
source install/setup.bash
```

### 3. Launch the Application

Start the simulation, clock bridge, spawner, and controller using a single launch file. By default, this opens in an empty Gazebo world:

```bash
ros2 launch noisy_boy_bringup catch_the_bot.launch.py
```
