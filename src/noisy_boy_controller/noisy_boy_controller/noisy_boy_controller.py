import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from noisy_boy_interfaces.msg import TargetBotArray
from noisy_boy_interfaces.srv import CatchBot


class NoisyBoyController(Node):
    def __init__(self):
        super().__init__("noisy_controller")
        self.target_bots = []
        self.catcher_x = 0.0
        self.catcher_y = 0.0
        self.catcher_yaw = 0.0
        self.is_catching = False
        self.catching_name = ""

        # Direct odom gives the SAME yaw the DiffDrive plugin uses internally,
        # making angular.z commands consistent with physical wheel motion.
        self.create_subscription(Odometry, "/robot1/ground_truth", self._odom_cb, 10)
        self.create_subscription(TargetBotArray, "target_bots", self._targets_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/robot1/cmd_vel", 10)
        self.catch_client = self.create_client(CatchBot, "catch_bot")
        self.create_timer(0.05, self._loop)
        self.get_logger().info("Controller ready")

    def _odom_cb(self, msg):
        self.catcher_x = msg.pose.pose.position.x
        self.catcher_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.catcher_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def _targets_cb(self, msg):
        self.target_bots = msg.target_bots

    def _loop(self):
        if not self.target_bots:
            self.cmd_vel_pub.publish(Twist())
            return

        if self.is_catching:
            if not any(b.name == self.catching_name for b in self.target_bots):
                self.is_catching = False
            else:
                self.cmd_vel_pub.publish(Twist())
            return

        nearest = min(self.target_bots,
                      key=lambda b: math.hypot(b.x - self.catcher_x, b.y - self.catcher_y))
        dist = math.hypot(nearest.x - self.catcher_x, nearest.y - self.catcher_y)

        if dist < 0.60:
            self.cmd_vel_pub.publish(Twist())
            self._catch(nearest.name)
            return

        raw_yaw = math.atan2(nearest.y - self.catcher_y, nearest.x - self.catcher_x)
        yaw_err = math.atan2(math.sin(raw_yaw - self.catcher_yaw),
                             math.cos(raw_yaw - self.catcher_yaw))

        KP_ANG, MAX_ANG = 1.5, 0.8
        KP_LIN, MAX_LIN = 0.5, 0.8

        twist = Twist()
        # Phase 1 — rotate in place until within ~20 deg of target
        if abs(yaw_err) > 0.35:
            twist.angular.z = max(-MAX_ANG, min(MAX_ANG, KP_ANG * yaw_err))
        # Phase 2 — drive forward with minor heading correction
        else:
            raw = KP_LIN * dist * (dist / 1.5 if dist < 1.5 else 1.0)
            twist.linear.x = min(MAX_LIN, max(0.0, raw))
            if abs(yaw_err) > 0.05:
                twist.angular.z = max(-MAX_ANG, min(MAX_ANG, KP_ANG * yaw_err))

        self.cmd_vel_pub.publish(twist)

    def _catch(self, name):
        if not self.catch_client.service_is_ready():
            return
        self.is_catching = True
        self.catching_name = name
        req = CatchBot.Request()
        req.name = name
        self.catch_client.call_async(req).add_done_callback(
            lambda f, n=name: self._catch_done(f, n))
        self.get_logger().info(f"Catching '{name}'")

    def _catch_done(self, future, name):
        try:
            if future.result().success:
                self.get_logger().info(f"Caught '{name}'")
            else:
                self.get_logger().warn(f"Miss: '{name}'")
        except Exception as e:
            self.get_logger().error(str(e))
        finally:
            self.is_catching = False
            self.catching_name = ""


def main(args=None):
    rclpy.init(args=args)
    node = NoisyBoyController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
