from math import hypot, atan2, inf, cos, sin, pi

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

        #只有朝向明显与前瞻点相反时，先原地对齐，不改动原始PP的前瞻距离和线速度
        self.declare_parameter("heading_alignment_enabled", True)
        self.declare_parameter("heading_enter_angle", 1.35)
        self.declare_parameter("heading_exit_angle", 0.25)
        self.declare_parameter("heading_turn_gain", 2.0)
        self.declare_parameter("heading_turn_max", 1.0)
        #只对角速度指令限制每秒变化量，正常前进仍使用原先的0.22m/s
        self.declare_parameter("angular_rate_limit_enabled", True)
        self.declare_parameter("angular_acceleration", 12.0)

        # Parameters: Get Values
        self.frequency_ = self.get_parameter("frequency").value
        self.lookahead_distance_ = self.get_parameter("lookahead_distance").value
        self.lookahead_lin_vel_ = self.get_parameter("lookahead_lin_vel").value
        self.stop_thres_ = self.get_parameter("stop_thres").value
        self.max_lin_vel_ = self.get_parameter("max_lin_vel").value
        self.max_ang_vel_ = self.get_parameter("max_ang_vel").value

        self.heading_alignment_enabled_ = bool(self.get_parameter("heading_alignment_enabled").value)
        self.heading_enter_angle_ = max(0.3, float(self.get_parameter("heading_enter_angle").value))
        self.heading_exit_angle_ = max(0.05, float(self.get_parameter("heading_exit_angle").value))
        if self.heading_exit_angle_ >= self.heading_enter_angle_:
            self.heading_exit_angle_ = 0.5 * self.heading_enter_angle_
        self.heading_turn_gain_ = max(0.1, float(self.get_parameter("heading_turn_gain").value))
        self.heading_turn_max_ = min(self.max_ang_vel_, max(0.1, float(self.get_parameter("heading_turn_max").value)))
        self.angular_rate_limit_enabled_ = bool(self.get_parameter("angular_rate_limit_enabled").value)
        self.angular_acceleration_ = max(0.01, float(self.get_parameter("angular_acceleration").value))
        self.heading_align_active_ = False
        self.last_cmd_w_ = 0.0
        self.last_cmd_stamp_ = None

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
        #算完速度之后从cmd_vel发给机器人
        self.pub_cmd_vel_ = self.create_publisher(TwistStamped, "cmd_vel", 10)

        # !TODO: lookahead point publisher
        #把前瞻点发出去，方便在RViz里看
        self.pub_lookahead_ = self.create_publisher(PoseStamped, "lookahead", 10)
        
        # Handles: Timers
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
        #把收到的整条路径留下来
        self.path_poses_ = []
        self.path_poses_ = list(msg.poses)

        self.received_path_ = True

    # Odometry subscriber callback
    def callbackSubOdom_(self, msg: Odometry):
        # !TODO: write robot pose to rbt_x_, rbt_y_, rbt_yaw_
        #x和y直接读位置，yaw要从四元数换算
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
        #这是机器人和前瞻点之间的实际距离
        dx = lookahead_x - self.rbt_x_
        dy = lookahead_y - self.rbt_y_
        distance = hypot(dx, dy)

        # stop the robot if close to the point.
        #判断是否到达终点，不能只看中间的前瞻点
        goal = self.path_poses_[-1].pose.position
        goal_distance = hypot(goal.x - self.rbt_x_, goal.y - self.rbt_y_)

        # get curvature
        #把前瞻点换到机器人坐标系，算出需要转弯的曲率
        local_y = -sin(self.rbt_yaw_) * dx + cos(self.rbt_yaw_) * dy
        curvature = 2.0 * local_y / max(distance * distance, 1e-9)

        # calculate velocities
        #先按设定的前进速度计算对应角速度
        requested_lin_vel = self.lookahead_lin_vel_
        requested_ang_vel = requested_lin_vel * curvature

        # saturate velocities. The following can result in the wrong curvature,
        # but only when the robot is travelling too fast (which should not occur if well tuned).
        lin_vel = 0.0
        ang_vel = 0.0 * lookahead_x * lookahead_y
        #保留老师原来的占位行，这里再填入真正的控制输出
        if goal_distance <= self.stop_thres_:
            #已到目标时必须立即停车，不为了平滑而延迟停止
            lin_vel = 0.0
            ang_vel = 0.0
            self.heading_align_active_ = False
            stop_now = True
        else:
            stop_now = False
            lin_vel = max(0.0, min(requested_lin_vel, self.max_lin_vel_))
            ang_vel = max(-self.max_ang_vel_, min(requested_ang_vel, self.max_ang_vel_))

            #大角度航向偏差：不向错误方向前进，先原地对准当前参考路径前瞻点
            heading_error = atan2(sin(atan2(dy, dx) - self.rbt_yaw_),
                                  cos(atan2(dy, dx) - self.rbt_yaw_))
            if self.heading_alignment_enabled_:
                if self.heading_align_active_:
                    if abs(heading_error) <= self.heading_exit_angle_:
                        self.heading_align_active_ = False
                elif abs(heading_error) >= self.heading_enter_angle_:
                    self.heading_align_active_ = True

                if self.heading_align_active_:
                    lin_vel = 0.0
                    ang_vel = max(-self.heading_turn_max_,
                                  min(self.heading_turn_max_, self.heading_turn_gain_ * heading_error))

            #限制指令角加速度，不缩减普通路段的线速度
            if self.angular_rate_limit_enabled_:
                now_ns = self.get_clock().now().nanoseconds
                if self.last_cmd_stamp_ is None:
                    dt = 1.0 / max(self.frequency_, 1.0)
                else:
                    dt = (now_ns - self.last_cmd_stamp_) * 1e-9
                    if dt <= 0.0 or dt > 0.5:
                        dt = 1.0 / max(self.frequency_, 1.0)
                max_delta = self.angular_acceleration_ * dt
                ang_vel = max(self.last_cmd_w_ - max_delta,
                              min(self.last_cmd_w_ + max_delta, ang_vel))

        #保存真正发出的角速度，下一帧用于变化率限制
        self.last_cmd_w_ = ang_vel
        self.last_cmd_stamp_ = self.get_clock().now().nanoseconds

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
