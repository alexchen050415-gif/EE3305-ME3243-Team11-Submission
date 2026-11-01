import math
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

        #保留原来Dijkstra，开关只决定要不要优化输出路径
        self.declare_parameter("path_optimization_enabled", True)
        self.declare_parameter("path_min_clearance", 0.06)
        self.declare_parameter("path_sample_step", 0.05)
        self.declare_parameter("path_smooth_iterations", 18)
        self.declare_parameter("path_time_turn_weight", 0.28)
        self.declare_parameter("path_time_clearance_weight", 0.12)
        self.declare_parameter("path_switch_gain", 0.03)

        # Parameters: Get Values
        self.max_access_cost_ = self.get_parameter("max_access_cost").value

        self.path_optimization_enabled_ = bool(self.get_parameter("path_optimization_enabled").value)
        self.path_min_clearance_ = float(self.get_parameter("path_min_clearance").value)
        self.path_sample_step_ = float(self.get_parameter("path_sample_step").value)
        self.path_smooth_iterations_ = int(self.get_parameter("path_smooth_iterations").value)
        self.path_time_turn_weight_ = float(self.get_parameter("path_time_turn_weight").value)
        self.path_time_clearance_weight_ = float(self.get_parameter("path_time_clearance_weight").value)
        self.path_switch_gain_ = float(self.get_parameter("path_switch_gain").value)
        self.last_opt_path_ = None
        self.last_opt_goal_ = None

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

    #读取路径点，后面只处理坐标，不碰ROS消息格式
    def pathCoords_(self, msg):
        return [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    #检查整条线段，不允许穿过机器人外轮廓附近的障碍物
    def segmentSafe_(self, a, b, cache=None):
        step = max(0.015, min(0.025, self.costmap_resolution_ * 0.5))
        distance = hypot(b[0]-a[0], b[1]-a[1])
        count = max(1, int(distance/step)+1)
        radius = max(0., self.path_min_clearance_)
        cell_radius = int(radius/self.costmap_resolution_)+2
        for i in range(count+1):
            t = i/count
            x,y = a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t
            c,r = self.XYToCR_(x,y)
            if self.outOfMap_(c,r):
                return False
            key = (c,r)
            if cache is not None and key in cache:
                if not cache[key]:return False
                continue
            good = True
            #膨胀代价高于阈值的格子不能穿，还额外考虑机器人半径
            for dr in range(-cell_radius,cell_radius+1):
                for dc in range(-cell_radius,cell_radius+1):
                    nc,nr=c+dc,r+dr
                    if self.outOfMap_(nc,nr):
                        good=False;break
                    dd=hypot(dc,dr)*self.costmap_resolution_
                    if dd<=radius+0.7072*self.costmap_resolution_:
                        val=self.costmap_[self.CRToIndex_(nc,nr)]
                        #cost=99相当于障碍物附近不可通行区域；未知直接挡住
                        if val < 0 or val >= 99:
                            good=False;break
                if not good:break
            if cache is not None:cache[key]=good
            if not good:return False
        return True

    #把长折线均匀重新采样，Pure Pursuit需要足够密的路径点
    def resamplePath_(self, pts):
        if len(pts)<2:return pts[:]
        dense=[pts[0]]
        step=max(0.03,self.path_sample_step_)
        for a,b in zip(pts[:-1],pts[1:]):
            length=hypot(b[0]-a[0],b[1]-a[1])
            n=max(1,int(math.ceil(length/step)))
            for i in range(1,n+1):
                t=i/n
                dense.append((a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])))
        return dense

    #沿着折线删掉不必要的中间格点
    def shortcutPath_(self, pts, cache):
        if len(pts)<3:return pts[:]
        picked=[pts[0]];i=0
        while i<len(pts)-1:
            #优先选最远且安全的点；不强制绕大弯
            nxt=i+1
            for j in range(len(pts)-1,i+1,-1):
                if self.segmentSafe_(pts[i],pts[j],cache):
                    nxt=j;break
            picked.append(pts[nxt]);i=nxt
        return picked

    #局部拉直折点，但每一步都做障碍物检测
    def smoothPath_(self, pts, cache):
        points=list(pts)
        for _ in range(max(0,self.path_smooth_iterations_)):
            old=points[:]
            for i in range(1,len(points)-1):
                a,b,c=points[i-1],points[i],points[i+1]
                p=(b[0]*0.65+(a[0]+c[0])*0.175,
                   b[1]*0.65+(a[1]+c[1])*0.175)
                if self.segmentSafe_(a,p,cache) and self.segmentSafe_(p,c,cache):
                    points[i]=p
            if max(hypot(a[0]-b[0],a[1]-b[1]) for a,b in zip(old,points))<1e-4:
                break
        return points

    #近似行驶时间：路长、急转次数和靠墙程度都参与评价
    def estimatedTime_(self, pts):
        if len(pts)<2:return float('inf')
        length=sum(hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(pts[:-1],pts[1:]))
        #Burger最高线速度0.22，转向增加必要的角度调整时间
        time_cost=length/0.22
        total_turn=0.0
        for a,b,c in zip(pts[:-2],pts[1:-1],pts[2:]):
            h1=math.atan2(b[1]-a[1],b[0]-a[0])
            h2=math.atan2(c[1]-b[1],c[0]-b[0])
            delta=abs(math.atan2(math.sin(h2-h1),math.cos(h2-h1)))
            total_turn+=delta
        time_cost+=self.path_time_turn_weight_*total_turn/2.84
        #适度保留净空偏好，但不会强制追求越远越好
        sample=pts[::max(1,len(pts)//60)]
        for x,y in sample:
            c,r=self.XYToCR_(x,y)
            if self.outOfMap_(c,r):return float('inf')
            v=self.costmap_[self.CRToIndex_(c,r)]
            if v<0 or v>=99:return float('inf')
            time_cost+=self.path_time_clearance_weight_*v/max(1,len(sample)*99)
        return time_cost

    #Dijkstra结果之后再选一条更顺、预计更快的安全路线
    def refinePath_(self, msg):
        if not self.path_optimization_enabled_:return msg
        source=self.pathCoords_(msg)
        if len(source)<3:return msg
        cache={}
        #原始路径作为保底候选；如果平滑不安全就不换
        candidates=[source]
        shortcuts=self.shortcutPath_(source,cache)
        if len(shortcuts)>=2:
            candidates.append(self.resamplePath_(shortcuts))
            candidates.append(self.smoothPath_(self.resamplePath_(shortcuts),cache))
        candidates.append(self.smoothPath_(source,cache))
        valid=[]
        for path in candidates:
            if all(self.segmentSafe_(a,b,cache) for a,b in zip(path[:-1],path[1:])):
                valid.append((self.estimatedTime_(path),path))
        if not valid:return msg
        valid.sort(key=lambda item:item[0])
        best_cost,best_path=valid[0]
        #每次重规划先检查旧路线，避免新路径只好一点点就来回切换
        goal=source[-1]
        if self.last_opt_path_ and self.last_opt_goal_==goal:
            old=self.last_opt_path_
            near=min(range(len(old)),key=lambda i:hypot(old[i][0]-source[0][0],old[i][1]-source[0][1]))
            remain=old[near:]
            if len(remain)>1 and self.segmentSafe_(source[0],remain[0],cache):
                retained=[source[0]]+remain
                if all(self.segmentSafe_(a,b,cache) for a,b in zip(retained[:-1],retained[1:])):
                    old_cost=self.estimatedTime_(retained)
                    if old_cost<=best_cost*(1+self.path_switch_gain_):
                        best_path=retained
        self.last_opt_goal_=goal
        self.last_opt_path_=best_path
        result=Path()
        result.header=msg.header
        for x,y in best_path:
            pose=PoseStamped()
            pose.pose.position.x=x
            pose.pose.position.y=y
            result.poses.append(pose)
        return result

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
                #只优化Dijkstra真正生成的路径，不影响老师原来的搜图流程
                msg_path = self.refinePath_(msg_path)
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
