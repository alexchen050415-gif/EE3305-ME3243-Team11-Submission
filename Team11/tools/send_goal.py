#!/usr/bin/env python3
"""直接向当前ROS_DOMAIN_ID发同一组目标坐标，避免两次RViz点击不一致。"""
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

def main():
    if len(sys.argv)!=3:
        raise SystemExit("Usage: python3 send_goal.py X Y")
    x,y=float(sys.argv[1]),float(sys.argv[2])
    rclpy.init()
    node=Node("ee3305_send_test_goal")
    pub=node.create_publisher(PoseStamped,"/goal_pose",10)
    msg=PoseStamped()
    msg.header.frame_id="map"
    msg.pose.position.x=x;msg.pose.position.y=y;msg.pose.orientation.w=1.0
    #等待连接，再发一次即可
    import time
    deadline=time.monotonic()+8
    while pub.get_subscription_count()==0 and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.2)
    msg.header.stamp=node.get_clock().now().to_msg()
    pub.publish(msg)
    rclpy.spin_once(node,timeout_sec=.4)
    print(f"Sent goal ({x:.3f}, {y:.3f}); subscribers={pub.get_subscription_count()}")
    node.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
