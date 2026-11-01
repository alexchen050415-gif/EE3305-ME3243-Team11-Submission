#!/usr/bin/env python3
"""独立只读记录器：不发目标、不改算法、不切换控制器。"""
import argparse
import base64
import zlib
import csv
import heapq
import json
import math
import os
import tempfile
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path as ROSPath

from compare_plot import update_plots

COLUMNS = ["t", "x", "y", "yaw", "v", "omega", "cmd_v", "cmd_omega",
           "distance", "clearance", "goal_distance"]


def xy_dist(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


def atom_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    with tmp.open("w", newline="", encoding="utf-8") as file:
        w = csv.DictWriter(file, fieldnames=COLUMNS)
        w.writeheader()
        w.writerows(rows)
    tmp.replace(path)


def atom_json(path, value):
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    tmp.replace(path)


def stamp(msg):
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


class Recorder(Node):
    def __init__(self, variant, output):
        super().__init__(f"ee3305_{variant.lower()}_recorder")
        self.variant = variant
        self.output = output
        self.output.mkdir(parents=True, exist_ok=True)
        self.last_odom = None
        self.last_cmd = (0., 0.)
        self.last_path = []
        self.grid = None
        self.distance_map = None
        self.occupancy_snapshot = None
        self.active = None
        self.last_stamp = None
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        self.create_subscription(TwistStamped, "/cmd_vel", self.on_cmd, 20)
        self.create_subscription(ROSPath, "/path", self.on_path, 10)
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)
        self.get_logger().info(f"Passive recorder ready ({variant}). Select a 2D Goal Pose.")

    def on_cmd(self, msg):
        self.last_cmd = (float(msg.twist.linear.x), float(msg.twist.angular.z))

    def on_path(self, msg):
        self.last_path = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]

    def on_map(self, msg):
        w, h = int(msg.info.width), int(msg.info.height)
        res = float(msg.info.resolution)
        if w*h != len(msg.data) or res <= 0:
            return
        self.grid = (w, h, res, float(msg.info.origin.position.x), float(msg.info.origin.position.y))
        #保存原始/map而不是膨胀后的Costmap，画图时才能区分真实障碍物与安全代价区
        q0 = msg.info.origin.orientation
        map_yaw = math.atan2(2*(q0.w*q0.z + q0.x*q0.y),
                             1-2*(q0.y*q0.y + q0.z*q0.z))
        raw_bytes = bytes(int(value)+1 for value in msg.data)
        self.occupancy_snapshot = dict(
            width=w, height=h, resolution=res,
            origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y), yaw=map_yaw,
            source="/map", encoding="zlib+base64 uint8(offset+1)",
            obstacle_threshold=65,
            data=base64.b64encode(zlib.compress(raw_bytes, 7)).decode("ascii"))
        #如果地图比Goal晚到，照样把地图补进本次结果
        if self.active is not None:
            self.save_map(self.active["group"])
        #这里算到已知障碍物的网格距离。未知区域不被当成已验证安全区域。
        source = [i for i,v in enumerate(msg.data) if v >= 65]
        dist = [float('inf')] * (w*h)
        q = []
        for i in source:
            dist[i] = 0.0
            q.append((0.0, i))
        heapq.heapify(q)
        moves = [(1,0,1.),(-1,0,1.),(0,1,1.),(0,-1,1.),
                 (1,1,2**.5),(1,-1,2**.5),(-1,1,2**.5),(-1,-1,2**.5)]
        while q:
            d, index = heapq.heappop(q)
            if d > dist[index] + 1e-9:
                continue
            r,c = divmod(index,w)
            for dc,dr,weight in moves:
                nc,nr = c+dc,r+dr
                if 0<=nc<w and 0<=nr<h:
                    j=nr*w+nc
                    nd=d+weight*res
                    if nd + 1e-9 < dist[j]:
                        dist[j]=nd
                        heapq.heappush(q,(nd,j))
        self.distance_map = dist
        self.get_logger().info("Map received; clearance estimator ready.")

    def save_map(self, group):
        if self.occupancy_snapshot is None:
            return
        #两个工作区共用同一张地图，仅需要保存一次
        dest = Path(group)/"map_occupancy.json"
        if not dest.exists():
            atom_json(dest, self.occupancy_snapshot)

    def pose(self,msg):
        p=msg.pose.pose.position
        q=msg.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        return (float(p.x),float(p.y),yaw)

    def clearance(self,x,y):
        if self.grid is None or self.distance_map is None:
            return float('nan')
        w,h,res,ox,oy=self.grid
        col=int(math.floor((x-ox)/res));row=int(math.floor((y-oy)/res))
        if not (0<=col<w and 0<=row<h):
            return -1.0
        border=min(x-ox,y-oy,ox+w*res-x,oy+h*res-y)
        #以Burger约0.115m外半径估算，另外减掉半格对角线误差
        return min(border,self.distance_map[row*w+col]-0.7072*res)-0.115

    def group_for(self,start,goal):
        #同一起终点自动放进同一文件夹，两个进程用文件锁防止同时建组冲突
        import fcntl
        lockpath=self.output / "group_index.lock"
        with lockpath.open("a+") as lock:
            fcntl.flock(lock.fileno(),fcntl.LOCK_EX)
            path=self.output / "group_index.json"
            try: groups=json.loads(path.read_text())
            except (FileNotFoundError,ValueError): groups=[]
            for group in reversed(groups):
                if xy_dist(group["start"], start)<.16 and xy_dist(group["goal"],goal)<.12:
                    return self.output / group["name"]
            folder=f"route_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"
            groups.append({"name":folder,"start":list(start),"goal":list(goal)})
            atom_json(path,groups)
            return self.output/folder

    def on_goal(self,msg):
        if self.last_odom is None:
            self.get_logger().warn("Goal arrived before odometry. Click the goal again.")
            return
        goal=(float(msg.pose.position.x),float(msg.pose.position.y))
        #有些发布器会重发同一个Goal，不要把一次实验拆成好几个
        if self.active is not None:
            if xy_dist(goal,self.active["goal"])<.005:
                return
            self.finish("new_goal_interrupt")
        start=self.pose(self.last_odom)
        group=self.group_for(start[:2],goal)
        group.mkdir(parents=True,exist_ok=True)
        self.save_map(group)
        ident=datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.active={"group":group,"ident":ident,"goal":goal,"start":list(start),
                     "rows":[],"t0":None,"last_xy":None,"length":0.,
                     "seen_move":False,"near_t":None,"still_t":None,
                     "last_save":0.,"path":list(self.last_path)}
        self.last_stamp=None
        self.get_logger().info(f"[{self.variant}] Recording goal {goal}; {group}")

    def on_odom(self,msg):
        self.last_odom=msg
        a=self.active
        if a is None:
            return
        s=stamp(msg)
        if s<=0:s=self.get_clock().now().nanoseconds*1e-9
        if a["t0"] is None:a["t0"]=s
        t=s-a["t0"]
        if t<-.1:
            self.finish("sim_clock_reset")
            return
        if self.last_stamp is not None and s-self.last_stamp<.075:
            return
        self.last_stamp=s
        x,y,yaw=self.pose(msg)
        v=math.hypot(float(msg.twist.twist.linear.x),float(msg.twist.twist.linear.y))
        omega=float(msg.twist.twist.angular.z)
        error=xy_dist((x,y),a["goal"])
        if a["last_xy"] is not None:
            a["length"]+=xy_dist((x,y),a["last_xy"])
        a["last_xy"]=(x,y)
        if a["length"]>.10:a["seen_move"]=True
        a["rows"].append(dict(t=round(t,4),x=x,y=y,yaw=yaw,v=v,omega=omega,
            cmd_v=self.last_cmd[0],cmd_omega=self.last_cmd[1],
            distance=a["length"],clearance=self.clearance(x,y),goal_distance=error))
        if self.last_path:
            a["path"]=list(self.last_path)
        if error<=.10 and v<.035:
            if a["near_t"] is None:a["near_t"]=t
            if t-a["near_t"]>=.5:
                self.finish("reached")
                return
        else:a["near_t"]=None
        if v<.012 and abs(omega)<.06 and abs(self.last_cmd[0])<.02:
            if a["still_t"] is None:a["still_t"]=t
            still=t-a["still_t"]
            if (error<=.18 and a["seen_move"] and still>2) or (a["seen_move"] and still>12) or (not a["seen_move"] and t>35):
                self.finish("near_goal_stopped" if error<=.18 else "stalled")
                return
        else:a["still_t"]=None
        if t>360:
            self.finish("timeout")
            return
        if t-a["last_save"]>4:
            atom_csv(a["group"]/f"{self.variant}_partial.csv",a["rows"])
            a["last_save"]=t

    def finish(self,status):
        a=self.active
        self.active=None
        if not a or not a["rows"]:return
        group=a["group"]
        self.save_map(group)
        csvname=f"{self.variant}_{a['ident']}.csv"
        atom_csv(group/csvname,a["rows"])
        partial=group/f"{self.variant}_partial.csv"
        if partial.exists():partial.unlink()
        last=a["rows"][-1]
        vals=[row["clearance"] for row in a["rows"] if math.isfinite(row["clearance"])]
        row=dict(variant=self.variant,status=status,elapsed_s=round(last["t"],3),
                 distance_m=round(a["length"],3),goal_error_m=round(last["goal_distance"],3),
                 min_clearance_m=round(min(vals),3) if vals else None,
                 csv=csvname,start=a["start"],goal=list(a["goal"]),
                 reference_path=a["path"])
        #用锁保护双项目同时完成时的写入；锁期间画图，避免两边覆盖同一张图
        import fcntl
        with (group/"records.lock").open("a+") as lock:
            fcntl.flock(lock.fileno(),fcntl.LOCK_EX)
            index=group/"records.json"
            try:rec=json.loads(index.read_text())
            except (FileNotFoundError,ValueError):rec=[]
            rec.append(row)
            atom_json(index,rec)
            try:
                result=update_plots(group)
            except Exception as ex:
                result=f"plots failed ({type(ex).__name__}: {ex}); CSV is saved"
        self.get_logger().info(f"[{self.variant}] {status}. Results: {group}. {result}")


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--variant",required=True,choices=["Original","Improved"])
    p.add_argument("--output",required=True)
    opts,rosargs=p.parse_known_args()
    import signal
    #脚本与ROS Launch分开运行，接到TERM也要保存已经走过的轨迹
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    rclpy.init(args=rosargs)
    node=Recorder(opts.variant,Path(opts.output).expanduser().resolve())
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.finish("manual_stop")
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=="__main__":main()
