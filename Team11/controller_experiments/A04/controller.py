from math import hypot, atan2, inf, cos, sin

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan


class Controller(Node):

    def __init__(self, node_name="controller"):
        # Node Constructor =============================================================
        super().__init__(node_name)

        # Parameters: Declare
        self.declare_parameter("frequency", float(20))
        self.declare_parameter("lookahead_distance", float(0.3))
        self.declare_parameter("lookahead_lin_vel", float(0.1))
        self.declare_parameter("stop_thres", float(0.1))
        self.declare_parameter("max_lin_vel", float(0.2))
        self.declare_parameter("max_ang_vel", float(2.0))

        # Parameters: Get Values
        self.frequency_ = self.get_parameter("frequency").value
        self.lookahead_distance_ = self.get_parameter("lookahead_distance").value
        self.lookahead_lin_vel_ = self.get_parameter("lookahead_lin_vel").value
        self.stop_thres_ = self.get_parameter("stop_thres").value
        self.max_lin_vel_ = self.get_parameter("max_lin_vel").value
        self.max_ang_vel_ = self.get_parameter("max_ang_vel").value

        # Handles: Topic Subscribers
        # !TODO: path subscriber
        #接收Planner发来的路径
        self.sub_path_ = self.create_subscription(
            Path, "path", self.callbackSubPath_, 10
        )

        # !TODO: odometry subscriber
        #读取机器人当前位置和朝向
        self.sub_odom_ = self.create_subscription(
            Odometry, "odom", self.callbackSubOdom_, 10
        )

        # Handles: Topic Publishers
        # !TODO: command velocities publisher
        self.pub_cmd_vel_ = self.create_publisher(TwistStamped, "cmd_vel", 10)

        # !TODO: lookahead point publisher
        self.pub_lookahead_ = self.create_publisher(PoseStamped, "lookahead", 10)
        
        # Handles: Timers
        self.prev_lin_vel_ = 0.0
        self.prev_cmd_time_ = None
        self.max_lin_acc_ = 0.50  # m/s^2
        self.timer = self.create_timer(1.0 / self.frequency_, self.callbackTimer_)

        # Other Instance Variables
        self.received_odom_ = False
        self.received_path_ = False

    # Callbacks =============================================================
    
    # Path subscriber callback
    def callbackSubPath_(self, msg: Path):
        if len(msg.poses) == 0:  # not msg.poses is fine but not clear
            self.get_logger().warn(f"Received path message is empty!")
            return  # do not update the path if no path is returned. This will ensure the copied path contains at least one point when the first non-empty path is received.

        # !TODO: copy the array from the path
        self.path_poses_ = []
        self.path_poses_ = list(msg.poses)

        self.received_path_ = True

    # Odometry subscriber callback
    def callbackSubOdom_(self, msg: Odometry):
        # !TODO: write robot pose to rbt_x_, rbt_y_, rbt_yaw_
        self.rbt_x_ = msg.pose.pose.position.x
        self.rbt_y_ = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.rbt_yaw_ = q.w
        self.rbt_yaw_ = atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        self.received_odom_ = True

    # Gets the lookahead point's coordinates based on the current robot's position and planner's path
    # Make sure path and robot positions are already received, and the path contains at least one point.
    def getLookaheadPoint_(self):
        # Find the point along the path that is closest to the robot
        #先看路径上哪个点离机器人最近
        closest_idx = min(
            range(len(self.path_poses_)),
            key=lambda i: hypot(
                self.path_poses_[i].pose.position.x - self.rbt_x_,
                self.path_poses_[i].pose.position.y - self.rbt_y_,
            ),
        )

        # From the closest point, iterate towards the goal and find the first point that is at least a lookahead distance away.
        # Return the goal point if no such lookahead point can be found
        lookahead_idx = len(self.path_poses_) - 1
        #从最近的点往后找，找到够远的点就作为前瞻点
        for i in range(closest_idx, len(self.path_poses_)):
            p = self.path_poses_[i].pose.position
            if hypot(p.x - self.rbt_x_, p.y - self.rbt_y_) >= self.lookahead_distance_:
                lookahead_idx = i
                break

        # Get the lookahead coordinates
        lookahead_pose = self.path_poses_[lookahead_idx]
        lookahead_x = lookahead_pose.pose.position.x
        lookahead_y = lookahead_pose.pose.position.y

        # Publish the lookahead coordinates
        msg_lookahead = PoseStamped()
        msg_lookahead.header.stamp = self.get_clock().now().to_msg()
        msg_lookahead.header.frame_id = "map"
        msg_lookahead.pose.position.x = lookahead_x
        msg_lookahead.pose.position.y = lookahead_y
        self.pub_lookahead_.publish(msg_lookahead)

        # Return the coordinates
        return lookahead_x, lookahead_y

    # Implement the pure pursuit controller here
    def callbackTimer_(self):
        if not self.received_odom_ or not self.received_path_:
            return  # return silently if path or odom is not received.

        # get lookahead point
        lookahead_x, lookahead_y = self.getLookaheadPoint_()

        # get distance to lookahead point (not to be confused with lookahead_distance)
        #机器人和前瞻点之间的实际距离
        dx = lookahead_x - self.rbt_x_
        dy = lookahead_y - self.rbt_y_
        distance = hypot(dx, dy)

        # stop the robot if close to the point.
        #判断是否到达终点
        goal = self.path_poses_[-1].pose.position
        goal_distance = hypot(goal.x - self.rbt_x_, goal.y - self.rbt_y_)

        # get curvature
        #把前瞻点换到机器人坐标系，算出转弯曲率
        local_y = -sin(self.rbt_yaw_) * dx + cos(self.rbt_yaw_) * dy
        curvature = 2.0 * local_y / max(distance * distance, 1e-9)

        # calculate velocities
        # Curvature-based adaptive velocity
        omega_target = 0.60  # rad/s

        requested_lin_vel = min(
            self.lookahead_lin_vel_,
            omega_target / max(abs(curvature), 1e-6)
        )
        requested_ang_vel = requested_lin_vel * curvature

        # saturate velocities. The following can result in the wrong curvature,
        # but only when the robot is travelling too fast (which should not occur if well tuned).
        lin_vel = 0.0
        ang_vel = 0.0 * lookahead_x * lookahead_y

        now = self.get_clock().now()
        if self.prev_cmd_time_ is None:
            dt = 1.0 / self.frequency_
        else:
            dt = (now - self.prev_cmd_time_).nanoseconds * 1e-9
            if dt <= 0.0 or dt > 0.2:
                dt = 1.0 / self.frequency_

        self.prev_cmd_time_ = now

        if goal_distance <= self.stop_thres_:
            lin_vel = 0.0
            ang_vel = 0.0
        else:
            target_vel = max(
                0.0,
                min(requested_lin_vel, self.max_lin_vel_)
            )

            # Limit acceleration, but allow immediate deceleration
            max_next_vel = self.prev_lin_vel_ + self.max_lin_acc_ * dt
            lin_vel = min(target_vel, max_next_vel)

            # Preserve Pure Pursuit curvature
            requested_ang_vel = lin_vel * curvature
            ang_vel = max(
                -self.max_ang_vel_,
                min(requested_ang_vel, self.max_ang_vel_)
            )

        self.prev_lin_vel_ = lin_vel

        # publish velocities
        msg_cmd_vel = TwistStamped()
        msg_cmd_vel.header.stamp = self.get_clock().now().to_msg()
        msg_cmd_vel.twist.linear.x = lin_vel
        msg_cmd_vel.twist.angular.z = ang_vel
        self.pub_cmd_vel_.publish(msg_cmd_vel)


# Main Boiler Plate =============================================================
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Controller())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
