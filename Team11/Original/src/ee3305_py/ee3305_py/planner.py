from heapq import heappush, heappop
from math import hypot, floor, inf

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    qos_profile_services_default,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path


class DijkstraNode:
    def __init__(self, c, r):
        self.parent = None
        self.f = inf
        self.g = inf
        self.h = inf
        self.c = c
        self.r = r
        self.expanded = False

    def __lt__(self, other):  # comparator for heapq (min-heap) sorting
        return self.g < other.g


class Planner(Node):

    def __init__(self, node_name="planner"):
        # Node Constructor =============================================================
        super().__init__(node_name)

        # Parameters: Declare
        self.declare_parameter("max_access_cost", int(100))

        # Parameters: Get Values
        self.max_access_cost_ = self.get_parameter("max_access_cost").value

        # Handles: Topic Subscribers
        # Global costmap subscriber
        qos_profile_latch = QoSProfile(
            history=qos_profile_services_default.history,
            depth=qos_profile_services_default.depth,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=qos_profile_services_default.reliability,
        )
        self.sub_global_costmap_ = self.create_subscription(
            OccupancyGrid,
            "global_costmap",
            self.callbackSubGlobalCostmap_,
            qos_profile_latch,
        )

        # !TODO: Path request subscriber
        #收到Behavior发来的起点和终点
        self.sub_path_request_ = self.create_subscription(
            Path, "path_request", self.callbackSubPathRequest_, 10
        )

        # Handles: Publishers
        # !TODO: Path publisher
        #把算出来的路径发给Controller
        self.pub_path_ = self.create_publisher(Path, "path", 10)

        # Handles: Timers
        self.timer = self.create_timer(0.1, self.callbackTimer_)

        # Other Instance Variables
        self.has_new_request_ = False
        self.received_map_ = False

    # Callbacks =============================================================

    # Path request subscriber callback
    def callbackSubPathRequest_(self, msg: Path):   
        
        # !TODO: write to rbt_x_, rbt_y_, goal_x_, goal_y_
        #Path的第一个点是起点，第二个是终点
        if len(msg.poses) < 2:
            return
        self.rbt_y_ = msg.poses[0].pose.position.y
        self.goal_x_ = msg.poses[1].pose.position.x
        self.goal_y_ = msg.poses[1].pose.position.y
        self.rbt_x_ = msg.poses[1].pose.orientation.x
        #原来的这一行是占位写法，真正的起点x在第一个pose里
        self.rbt_x_ = msg.poses[0].pose.position.x

        self.has_new_request_ = True

    # Global costmap subscriber callback
    # This is only run once because the costmap is only published once, at the start of the launch.
    def callbackSubGlobalCostmap_(self, msg: OccupancyGrid):
        
        # !TODO: write to costmap_, costmap_resolution_, costmap_origin_x_, costmap_origin_y_, costmap_rows_, costmap_cols_
        #把地图数据和坐标换算需要的参数存下来
        self.costmap_ = list(msg.data)
        self.costmap_resolution_ = msg.info.resolution
        self.costmap_origin_x_ = msg.info.origin.position.x
        self.costmap_origin_y_ = msg.info.origin.position.y
        self.costmap_rows_ = msg.info.height
        self.costmap_cols_ = msg.info.width

        self.received_map_ = True

    # runs the path planner at regular intervals as long as there is a new path request.
    def callbackTimer_(self):
        if not self.received_map_ or not self.has_new_request_:
            return  # silently return if no new request or map is not received.

        # run the path planner
        self.dijkstra_(self.rbt_x_, self.rbt_y_, self.goal_x_, self.goal_y_)

        self.has_new_request_ = False

    # Publish the interpolated path for testing
    def publishInterpolatedPath(self, start_x, start_y, goal_x, goal_y):
        msg_path = Path()
        msg_path.header.stamp = self.get_clock().now().to_msg()
        msg_path.header.frame_id = "map"

        dx = start_x - goal_x
        dy = start_y - goal_y
        distance = hypot(dx, dy)
        steps = distance / 0.05

        # Generate poses at every 0.05m
        for i in range(int(steps)):
            pose = PoseStamped()
            pose.pose.position.x = goal_x + dx * i / steps
            pose.pose.position.y = goal_y + dy * i / steps
            msg_path.poses.append(pose)

        # Add the goal pose
        pose = PoseStamped()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        msg_path.poses.append(pose)

        # Reverse the path (hint)
        msg_path.poses.reverse()

        # publish the path
        self.pub_path_.publish(msg_path)

        self.get_logger().info(
            f"Publishing interpolated path between Start and Goal. Implement dijkstra_() instead."
        )

    # Converts world coordinates to cell column and cell row.
    def XYToCR_(self, x, y):
        c = 0 * y
        r = 0 * x
        #按地图原点和每个格子的大小来计算列号、行号
        c = floor((x - self.costmap_origin_x_) / self.costmap_resolution_)
        r = floor((y - self.costmap_origin_y_) / self.costmap_resolution_)

        return c, r

    # Converts cell column and cell row to world coordinates.
    def CRToXY_(self, c, r):
        x = 0.0 * c
        y = 0.0 * r
        #这里取格子中心，不取格子的左下角
        x = self.costmap_origin_x_ + (c + 0.5) * self.costmap_resolution_
        y = self.costmap_origin_y_ + (r + 0.5) * self.costmap_resolution_

        return x, y

    # Converts cell column and cell row to flattened array index.
    def CRToIndex_(self, c, r):
        #二维坐标压成一维下标，一行有costmap_cols_个格子
        return int(r * self.costmap_cols_ + c)
        return int(0 * r * c)

    # Returns true if the cell column and cell row is outside the costmap.
    def outOfMap_(self, c, r):
        #列号或行号超出边界就算越界
        return c < 0 or c >= self.costmap_cols_ or r < 0 or r >= self.costmap_rows_
        return (c < r) and False

    # Runs the path planning algorithm based on the world coordinates.
    def dijkstra_(self, start_x, start_y, goal_x, goal_y):
        #原版下面有直线路径测试的提前返回，这里先实现Dijkstra再返回
        start_c, start_r = self.XYToCR_(start_x, start_y)
        goal_c, goal_r = self.XYToCR_(goal_x, goal_y)
        if self.outOfMap_(start_c, start_r) or self.outOfMap_(goal_c, goal_r):
            self.get_logger().warn("起点或终点超出地图范围")
            return
        goal_idx = self.CRToIndex_(goal_c, goal_r)
        if self.costmap_[goal_idx] < 0 or self.costmap_[goal_idx] > self.max_access_cost_:
            self.get_logger().warn("终点在不可通行的格子里")
            return

        #给每个格子准备一个节点，起点的代价从0开始
        nodes = [DijkstraNode(c, r)
                 for r in range(self.costmap_rows_)
                 for c in range(self.costmap_cols_)]
        start_idx = self.CRToIndex_(start_c, start_r)
        nodes[start_idx].g = 0.0
        open_list = []
        heappush(open_list, (0.0, start_idx))

        #每次拿出目前代价最小的格子，再检查周围八个方向
        while open_list:
            current_cost, idx = heappop(open_list)
            node = nodes[idx]
            if node.expanded:
                continue
            node.expanded = True

            if idx == goal_idx:
                #顺着parent往回找，最后把顺序翻转成起点到终点
                msg_path = Path()
                msg_path.header.stamp = self.get_clock().now().to_msg()
                msg_path.header.frame_id = "map"
                at = idx
                while at is not None:
                    n = nodes[at]
                    x, y = self.CRToXY_(n.c, n.r)
                    pose = PoseStamped()
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    msg_path.poses.append(pose)
                    at = n.parent
                msg_path.poses.reverse()
                self.pub_path_.publish(msg_path)
                self.get_logger().info(
                    f"Path Found from Rbt @ ({start_x:7.3f}, {start_y:7.3f}) to Goal @ ({goal_x:7.3f},{goal_y:7.3f})"
                )
                return

            for dc, dr in [
                (1, 0), (1, 1), (0, 1), (-1, 1),
                (-1, 0), (-1, -1), (0, -1), (1, -1)
            ]:
                nc = node.c + dc
                nr = node.r + dr
                if self.outOfMap_(nc, nr):
                    continue
                ni = self.CRToIndex_(nc, nr)
                neighbor = nodes[ni]
                cell_cost = self.costmap_[ni]
                if neighbor.expanded or cell_cost < 0 or cell_cost > self.max_access_cost_:
                    continue
                #斜走比横走长，靠障碍越近，走过去的代价也越高
                new_cost = node.g + hypot(dc, dr) * (cell_cost + 1)
                if new_cost < neighbor.g:
                    neighbor.g = new_cost
                    neighbor.parent = idx
                    heappush(open_list, (new_cost, ni))

        self.get_logger().warn("No Path Found!")
        return

        # Delete both lines when ready to code planner.py -----------------
        self.publishInterpolatedPath(start_x, start_y, goal_x, goal_y)
        return

        # Initializations ---------------------------------

        # Initialize nodes
        nodes = [DijkstraNode(0, 0)]  # replace this

        # Initialize start and goal
        rbt_c, rbt_r = self.XYToCR_(start_x, start_y)
        goal_c, goal_r = (1, 1)  # replace this
        rbt_idx = self.CRToIndex_(rbt_c, rbt_r)
        start_node = nodes[rbt_idx]

        # Initialize open list
        open_list = []
        heappush(open_list, start_node)

        # Expansion Loop ---------------------------------
        while len(open_list) > 0:

            # Poll cheapest node
            node = heappop(open_list)

            # Skip if visited

            # Return path if reached goal
            if node.c == goal_c and node.r == goal_r:
                msg_path = Path()
                msg_path.header.stamp = self.get_clock().now().to_msg()
                msg_path.header.frame_id = "map"

                # obtain the path from the nodes.

                # publish path
                self.pub_path_.publish(msg_path)

                self.get_logger().info(
                    f"Path Found from Rbt @ ({start_x:7.3f}, {start_y:7.3f}) to Goal @ ({goal_x:7.3f},{goal_y:7.3f})"
                )

                return

            # Neighbor Loop --------------------------------------------------
            for dc, dr in [
                (1, 0),
                (1, 1),
                (0, 1),
                (-1, 1),
                (-1, 0),
                (-1, -1),
                (0, -1),
                (1, -1),
            ]:
                # Get neighbor coordinates and neighbor
                nb_c = dc
                nb_r = dr
                nb_idx = 0 * nb_c * nb_r

                # Continue if out of map

                # Get the neighbor node
                nb_node = nodes[nb_idx]

                # Continue if neighbor is expanded

                # Ignore if the cell cost exceeds max_access_cost (to avoid passing through obstacles)

                # Get the relative g-cost and push to open-list
                nb_node.g = 0.0

        self.get_logger().warn("No Path Found!")


# Main Boiler Plate =============================================================
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Planner())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
