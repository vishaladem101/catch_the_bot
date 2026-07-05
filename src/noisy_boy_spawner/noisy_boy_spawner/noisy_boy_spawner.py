import os
import random
import subprocess
import rclpy
import signal
import time
import math
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf2_ros
from tf2_ros import TransformException
from std_srvs.srv import Trigger
from noisy_boy_interfaces.msg import TargetBot, TargetBotArray
from noisy_boy_interfaces.srv import CatchBot
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry

class NoisyBoySpawner(Node):

    def __init__(self):
        super().__init__("noisy_spawner")

        # Parameters
        self.declare_parameter("world_name", "empty")
        self.world_name = self.get_parameter("world_name").get_parameter_value().string_value

        # Track robot indices and spawned subprocesses mapped to robot names
        self.robot_count = 0
        self.robot_processes = {}
        self.static_transforms = []
        self.alive_target_bots = TargetBotArray()
        self.pending_bots = []
        self.catcher_x = 0.0
        self.catcher_y = 0.0

        # Subscribe to catcher robot's ground truth odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/robot1/ground_truth',
            self.catcher_odom_callback,
            10
        )

        # Create static TF broadcaster to stitch odom frames to the map frame
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Create service to trigger spawning of subsequent robots on-demand
        self.srv = self.create_service(Trigger, 'spawn_robot', self.spawn_robot_callback)

        # Create service to catch/kill robots
        self.catch_srv = self.create_service(CatchBot, 'catch_bot', self.catch_bot_callback)

        # Create publisher for the list of active target bots
        self.target_bots_pub = self.create_publisher(TargetBotArray, 'target_bots', 10)

        # Create publisher for target bot visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, 'target_bot_markers', 10)

        # Setup a timer to check if Gazebo service is ready before spawning the first robot
        self.check_gazebo_timer = self.create_timer(0.5, self.wait_for_gazebo)

        # Timer to check and activate fully spawned robots
        self.activation_timer = self.create_timer(0.1, self.check_pending_bots)

        # Timer to publish markers periodically at 1Hz
        self.marker_timer = self.create_timer(1.0, self.publish_markers)

        self.get_logger().info("Multi-robot spawner node initialized. Services /spawn_robot and /catch_bot are ready.")

    def check_pending_bots(self):
        now = self.get_clock().now()
        activated = False
        remaining_pending = []
        for activation_time, bot in self.pending_bots:
            if now >= activation_time:
                self.alive_target_bots.target_bots.append(bot)
                activated = True
                self.get_logger().info(f"Target bot {bot.name} is now fully spawned and active!")
            else:
                remaining_pending.append((activation_time, bot))
        self.pending_bots = remaining_pending
        if activated:
            self.target_bots_pub.publish(self.alive_target_bots)
            self.publish_markers()

    def publish_markers(self):
        marker_array = MarkerArray()

        # Now add visual markers for each alive target bot
        for bot in self.alive_target_bots.target_bots:
            # 1. Main body (Red Box)
            body = Marker()
            body.header.frame_id = "map"
            body.header.stamp = self.get_clock().now().to_msg()
            body.ns = bot.name
            body.id = 0
            body.type = Marker.CUBE
            body.action = Marker.ADD
            body.pose.position.x = bot.x
            body.pose.position.y = bot.y
            body.pose.position.z = 0.15  # base_joint(0.1) + half of box height(0.05)
            body.pose.orientation.w = 1.0
            body.scale.x = 0.4
            body.scale.y = 0.4
            body.scale.z = 0.1
            body.color.r = 1.0
            body.color.g = 0.0
            body.color.b = 0.0
            body.color.a = 1.0
            marker_array.markers.append(body)

            # 2. Left Wheel (Grey Cylinder)
            left_wheel = Marker()
            left_wheel.header.frame_id = "map"
            left_wheel.header.stamp = self.get_clock().now().to_msg()
            left_wheel.ns = bot.name
            left_wheel.id = 1
            left_wheel.type = Marker.CYLINDER
            left_wheel.action = Marker.ADD
            left_wheel.pose.position.x = bot.x
            left_wheel.pose.position.y = bot.y + 0.225  # width/2 + wheel_width/2
            left_wheel.pose.position.z = 0.1
            # 90 degrees rotation around X axis (roll = pi/2)
            left_wheel.pose.orientation.x = 0.7071
            left_wheel.pose.orientation.y = 0.0
            left_wheel.pose.orientation.z = 0.0
            left_wheel.pose.orientation.w = 0.7071
            left_wheel.scale.x = 0.2  # diameter (2 * radius)
            left_wheel.scale.y = 0.2
            left_wheel.scale.z = 0.05  # length/width of wheel
            left_wheel.color.r = 0.5
            left_wheel.color.g = 0.5
            left_wheel.color.b = 0.5
            left_wheel.color.a = 1.0
            marker_array.markers.append(left_wheel)

            # 3. Right Wheel (Grey Cylinder)
            right_wheel = Marker()
            right_wheel.header.frame_id = "map"
            right_wheel.header.stamp = self.get_clock().now().to_msg()
            right_wheel.ns = bot.name
            right_wheel.id = 2
            right_wheel.type = Marker.CYLINDER
            right_wheel.action = Marker.ADD
            right_wheel.pose.position.x = bot.x
            right_wheel.pose.position.y = bot.y - 0.225
            right_wheel.pose.position.z = 0.1
            right_wheel.pose.orientation.x = 0.7071
            right_wheel.pose.orientation.y = 0.0
            right_wheel.pose.orientation.z = 0.0
            right_wheel.pose.orientation.w = 0.7071
            right_wheel.scale.x = 0.2
            right_wheel.scale.y = 0.2
            right_wheel.scale.z = 0.05
            right_wheel.color.r = 0.5
            right_wheel.color.g = 0.5
            right_wheel.color.b = 0.5
            right_wheel.color.a = 1.0
            marker_array.markers.append(right_wheel)

            # 4. Lidar (Blue Cylinder)
            lidar = Marker()
            lidar.header.frame_id = "map"
            lidar.header.stamp = self.get_clock().now().to_msg()
            lidar.ns = bot.name
            lidar.id = 3
            lidar.type = Marker.CYLINDER
            lidar.action = Marker.ADD
            lidar.pose.position.x = bot.x
            lidar.pose.position.y = bot.y
            lidar.pose.position.z = 0.25  # base_joint(0.1) + base_box(0.1) + lidar_length/2(0.05)
            lidar.pose.orientation.w = 1.0
            lidar.scale.x = 0.2  # diameter (2 * radius)
            lidar.scale.y = 0.2
            lidar.scale.z = 0.1  # length
            lidar.color.r = 0.0
            lidar.color.g = 0.0
            lidar.color.b = 1.0
            lidar.color.a = 1.0
            marker_array.markers.append(lidar)

        if marker_array.markers:
            self.marker_pub.publish(marker_array)

    def publish_delete_markers(self, robot_name):
        marker_array = MarkerArray()
        for i in range(4): # body, left_wheel, right_wheel, lidar
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = robot_name
            m.id = i
            m.action = Marker.DELETE
            marker_array.markers.append(m)
        self.marker_pub.publish(marker_array)

    def wait_for_gazebo(self):
        if self.count_publishers('/clock') > 0:
            self.get_logger().info("Gazebo clock publisher detected. Simulation is ready! Spawning first robot.")
            self.check_gazebo_timer.cancel()
            self.spawn_first_robot()

    def catcher_odom_callback(self, msg):
        """
        Use ground truth odometry to get the exact physical position in Gazebo.
        """
        self.catcher_x = msg.pose.pose.position.x
        self.catcher_y = msg.pose.pose.position.y

    def generate_valid_coordinates(self):
        # Rejection sampling to find a target coordinate that is between 2m and 10m
        # from the catcher robot and within the [-6.0, 6.0] arena boundary
        self.get_logger().info(f"Generating coordinates. Catcher is at x={self.catcher_x:.2f}, y={self.catcher_y:.2f}")
        for _ in range(100):
            x = random.uniform(-6.0, 6.0)
            y = random.uniform(-6.0, 6.0)
            dx = x - self.catcher_x
            dy = y - self.catcher_y
            dist = math.sqrt(dx*dx + dy*dy)
            if 2.0 <= dist <= 10.0:
                return x, y

        # Fallback if no point is found (extremely unlikely)
        theta = random.uniform(0.0, 2.0 * math.pi)
        x = self.catcher_x + 4.0 * math.cos(theta)
        y = self.catcher_y + 4.0 * math.sin(theta)
        x = max(-6.0, min(6.0, x))
        y = max(-6.0, min(6.0, y))
        return x, y

    def spawn_first_robot(self):
        self.get_logger().info("Spawning first robot (robot1) on startup...")
        self.spawn_robot_at(x=0.0, y=0.0)
        # Spawn the first target bot automatically
        x, y = self.generate_valid_coordinates()
        self.spawn_robot_at(x=x, y=y)

    def spawn_robot_callback(self, request, response):
        self.get_logger().info("Spawn robot service triggered.")
        # Generate random coordinates for subsequent robots
        x, y = self.generate_valid_coordinates()
        robot_name = self.spawn_robot_at(x=x, y=y)
        response.success = True
        response.message = f"Successfully spawned {robot_name} at x={x:.2f}, y={y:.2f}"
        return response

    def spawn_robot_at(self, x, y):
        # 1. Generate unique namespace
        self.robot_count += 1
        robot_name = f"robot{self.robot_count}"
        self.get_logger().info(f"Preparing to spawn {robot_name} at x={x:.2f}, y={y:.2f}")

        # 2. Retrieve URDF/Xacro path
        pkg_noisy_boy_description = get_package_share_directory('noisy_boy_description')
        xacro_path = os.path.join(pkg_noisy_boy_description, 'urdf', 'noisy_boy.xacro')

        # 3. Generate the namespaced URDF string
        try:
            cmd = ["xacro", xacro_path, f"namespace:={robot_name}"]
            urdf_content = subprocess.check_output(cmd, encoding='utf-8')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to run xacro: {e}")
            return robot_name

        # 4. Start Namespaced Robot State Publisher
        rsp_cmd = [
            "ros2", "run", "robot_state_publisher", "robot_state_publisher",
            "--ros-args",
            "-r", f"__ns:=/{robot_name}",
            "-p", f"frame_prefix:={robot_name}/",
            "-p", f"robot_description:={urdf_content}",
            "-p", "use_sim_time:=true"
        ]
        # Use preexec_fn=os.setsid to put the subprocess in its own process group
        rsp_proc = subprocess.Popen(rsp_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

        # 5. Start Namespaced Parameter Bridge
        bridge_args = [
            f"/model/{robot_name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            f"/model/{robot_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            f"/model/{robot_name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            f"/model/{robot_name}/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            f"/model/{robot_name}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            f"/model/{robot_name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ]
        bridge_cmd = [
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            *bridge_args,
            "--ros-args",
            "-r", f"/model/{robot_name}/tf:=/tf",
            "-r", f"/model/{robot_name}/cmd_vel:=/{robot_name}/cmd_vel",
            "-r", f"/model/{robot_name}/odom:=/{robot_name}/odom",
            "-r", f"/model/{robot_name}/ground_truth:=/{robot_name}/ground_truth",
            "-r", f"/model/{robot_name}/joint_states:=/{robot_name}/joint_states",
            "-r", f"/model/{robot_name}/scan:=/{robot_name}/scan",
            "-p", "use_sim_time:=true"
        ]
        bridge_proc = subprocess.Popen(bridge_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

        # 6. Spawn model in Gazebo Sim synchronously
        spawn_cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-name", robot_name,
            "-x", f"{x:.2f}",
            "-y", f"{y:.2f}",
            "-z", "0.1",
            "-topic", f"/{robot_name}/robot_description"
        ]
        try:
            subprocess.run(spawn_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
            self.get_logger().info(f"Successfully spawned {robot_name} in Gazebo.")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to spawn {robot_name} in Gazebo: {e}")

        # Track processes for this robot
        self.robot_processes[robot_name] = [rsp_proc, bridge_proc]

        # If it is not the main catcher robot, add it to self.pending_bots to activate after a delay
        if robot_name != 'robot1':
            bot = TargetBot()
            bot.name = robot_name
            bot.x = x
            bot.y = y
            bot.theta = 0.0
            # Set activation time to 2.0 seconds in the future
            activation_time = self.get_clock().now() + rclpy.duration.Duration(seconds=2.0)
            self.pending_bots.append((activation_time, bot))
            self.get_logger().info(f"Scheduled {robot_name} to activate in 2.0 seconds...")

        # 7. Add TF from 'map' to 'robotX/odom'
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = f'{robot_name}/odom'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0  # Identity rotation (aligned orientation)

        # Keep a history of all transforms, and re-broadcast the entire array
        # to prevent older transforms from being overwritten on the latched /tf_static topic
        self.static_transforms.append(t)
        self.tf_broadcaster.sendTransform(self.static_transforms)
        self.get_logger().info(f"Broadcasted static TF: map -> {robot_name}/odom")

        return robot_name

    def catch_bot_callback(self, request: CatchBot.Request, response: CatchBot.Response):
        self.get_logger().info(f"Catch robot service triggered for '{request.name}'")

        target_bot = None
        for bot in self.alive_target_bots.target_bots:
            if bot.name == request.name:
                target_bot = bot
                break

        if target_bot is not None:
            # 1. Remove from active list
            self.alive_target_bots.target_bots.remove(target_bot)
            self.target_bots_pub.publish(self.alive_target_bots)
            self.publish_delete_markers(request.name)
            self.publish_markers()
            self.get_logger().info(f"Removed '{request.name}' from active target bots list")

            # 2. Terminate background processes cleanly
            if request.name in self.robot_processes:
                processes = self.robot_processes[request.name]
                self.get_logger().info(f"Terminating processes for '{request.name}'")
                for proc in processes:
                    try:
                        pgid = os.getpgid(proc.pid)
                        os.killpg(pgid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
                    except Exception as e:
                        self.get_logger().warn(f"Failed to SIGINT pgid {proc.pid}: {e}")

                time.sleep(0.3)
                for proc in processes:
                    try:
                        pgid = os.getpgid(proc.pid)
                        os.killpg(pgid, signal.SIGKILL)
                        proc.wait()
                    except ProcessLookupError:
                        pass
                    except Exception as e:
                        pass
                del self.robot_processes[request.name]

            # 3. Call Gazebo service directly to delete from Gazebo
            remove_cmd = [
                "gz", "service", "-s", f"/world/{self.world_name}/remove",
                "--reqtype", "gz.msgs.Entity",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "2000",
                "--req", f"type: 2, name: '{request.name}'"
            ]
            try:
                subprocess.Popen(remove_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                self.get_logger().info(f"Sent Gazebo service remove command for '{request.name}'")
            except Exception as e:
                self.get_logger().error(f"Failed to run gz service remove command for '{request.name}': {e}")

            # 4. Remove TF from self.static_transforms
            new_transforms = []
            for t in self.static_transforms:
                if t.child_frame_id != f"{request.name}/odom":
                    new_transforms.append(t)
            self.static_transforms = new_transforms
            self.tf_broadcaster.sendTransform(self.static_transforms)
            self.get_logger().info(f"Updated static TFs after removing '{request.name}'")

            # 4. Spawn a new target bot automatically to replace the caught one
            x, y = self.generate_valid_coordinates()
            self.spawn_robot_at(x=x, y=y)

            response.success = True
        else:
            response.success = False
            self.get_logger().warn(f"Catch robot requested for '{request.name}', but it was not found in active target bots list")

        return response

    def destroy_node(self):
        # Terminate all background nodes cleanly when this node shuts down
        self.get_logger().info("Shutting down spawner and cleaning up background processes...")

        all_procs = []
        for procs in self.robot_processes.values():
            all_procs.extend(procs)

        # 1. Send SIGINT (Ctrl+C) to all process groups
        for proc in all_procs:
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGINT)
            except ProcessLookupError:
                pass
            except Exception as e:
                self.get_logger().warn(f"Failed to send SIGINT to pgid {proc.pid}: {e}")

        # 2. Wait briefly for processes to stop gracefully
        time.sleep(0.5)

        # 3. Force kill (SIGKILL) any remaining processes
        for proc in all_procs:
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGKILL)
                proc.wait()
            except ProcessLookupError:
                pass
            except Exception as e:
                self.get_logger().warn(f"Failed to SIGKILL pgid {proc.pid}: {e}")

        super().destroy_node()

def main():
    rclpy.init()
    node = NoisyBoySpawner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    except Exception as e:
        print(f"Exception in spawner node: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
