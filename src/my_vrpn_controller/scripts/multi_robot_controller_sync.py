#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import PoseStamped, Twist, Pose
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool, Int32 # <<< NEW: 导入新的消息类型

class GenericRobotController:
    def __init__(self):
        # ... (初始化、参数加载等部分保持不变) ...
        rospy.init_node('generic_robot_controller', anonymous=True)
        self.robot_name = rospy.get_param('~robot_name', 'default_robot')
        vrpn_topic_name = f"/vrpn_client_node/{self.robot_name}/pose"
        waypoints_param = rospy.get_param('~waypoints', [[0.0, 0.0]])
        self.waypoints = [tuple(p) for p in waypoints_param]
        self.distance_tolerance = rospy.get_param('~distance_tolerance', 150.0)
        self.angle_tolerance = rospy.get_param('~angle_tolerance', 0.2)
        self.linear_speed_gain = rospy.get_param('~linear_speed_gain', 0.4)
        self.angular_speed_gain = rospy.get_param('~angular_speed_gain', 1.0)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 800.0)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.8)
        self.yaw_offset = rospy.get_param('~yaw_offset', 0.0)

        # <<< NEW: 修改状态变量 >>>
        self.waypoint_index = -1 # 初始为-1，等待协调器指令
        self.goal_x = 0
        self.goal_y = 0
        self.is_active = False # 机器人默认不移动，等待指令
        self.all_goals_reached = False

        # --- ROS发布者和订阅者 ---
        # <<< MODIFIED: 话题名称现在是相对的，以便命名空间生效 >>>
        self.status_pub = rospy.Publisher('waypoint_reached', Bool, queue_size=1)
        # <<< MODIFIED: cmd_vel也使用相对名称 >>>
        cmd_vel_topic_name = "cmd_vel"
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic_name, Twist, queue_size=10)
        
        self.vrpn_sub = rospy.Subscriber(vrpn_topic_name, PoseStamped, self.pose_callback)
        # <<< NEW: 订阅来自协调器的“下一个路点”指令 >>>
        self.next_waypoint_sub = rospy.Subscriber('/mission_control/next_waypoint', Int32, self.next_waypoint_callback)
        
        rospy.loginfo(f"[{self.robot_name}] Synchronized controller ready. Waiting for first command from coordinator.")

    def next_waypoint_callback(self, msg):
        """接收到协调器的指令后，更新目标路点"""
        new_index = msg.data
        if new_index >= len(self.waypoints):
            self.all_goals_reached = True
            self.is_active = False
            rospy.loginfo(f"[{self.robot_name}] All waypoints completed. Mission finished.")
            # 发送一次停止指令
            self.cmd_vel_pub.publish(Twist())
            return

        if new_index > self.waypoint_index:
            self.waypoint_index = new_index
            self.goal_x = self.waypoints[self.waypoint_index][0]
            self.goal_y = self.waypoints[self.waypoint_index][1]
            self.is_active = True # 激活机器人，允许移动
            rospy.loginfo(f"[{self.robot_name}] Received command. Heading to waypoint #{self.waypoint_index + 1}: x={self.goal_x:.2f}, y={self.goal_y:.2f}")

    def pose_callback(self, msg):
        # <<< MODIFIED: 只有在激活状态下才执行控制逻辑 >>>
        if self.is_active and not self.all_goals_reached:
            self.control_loop(msg.pose)

    def control_loop(self, current_pose):
        # ... (计算和移动的逻辑几乎不变) ...
        current_x = current_pose.position.x
        current_y = current_pose.position.y
        orientation_q = current_pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        calibrated_yaw = yaw + self.yaw_offset
        if calibrated_yaw > math.pi: calibrated_yaw -= 2 * math.pi
        elif calibrated_yaw < -math.pi: calibrated_yaw += 2 * math.pi

        distance_to_goal = math.sqrt((self.goal_x - current_x)**2 + (self.goal_y - current_y)**2)
        angle_to_goal = math.atan2(self.goal_y - current_y, self.goal_x - current_x)
        angle_error = angle_to_goal - calibrated_yaw
        if angle_error > math.pi: angle_error -= 2 * math.pi
        elif angle_error < -math.pi: angle_error += 2 * math.pi
        
        cmd_vel_msg = Twist()
        
        # <<< MODIFIED: 到达目标后的逻辑 >>>
        if distance_to_goal < self.distance_tolerance:
            rospy.loginfo(f"[{self.robot_name}] Waypoint #{self.waypoint_index + 1} reached! Stopping and reporting to coordinator.")
            self.is_active = False # 停止移动
            self.cmd_vel_pub.publish(Twist()) # 发布停止指令
            self.status_pub.publish(Bool(data=True)) # 向协调器报告“我已完成”
            return # 退出控制循环，原地等待新指令
        
        # 移动逻辑不变
        if abs(angle_error) > self.angle_tolerance:
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = self.angular_speed_gain * angle_error
        else:
            expected_speed_mms = self.linear_speed_gain * distance_to_goal
            cmd_vel_msg.linear.x = expected_speed_mms / 1000.0
            cmd_vel_msg.angular.z = self.angular_speed_gain * angle_error
        
        max_linear_speed_ms = self.max_linear_speed / 1000.0
        cmd_vel_msg.linear.x = min(cmd_vel_msg.linear.x, max_linear_speed_ms)
        cmd_vel_msg.angular.z = max(min(cmd_vel_msg.angular.z, self.max_angular_speed), -self.max_angular_speed)
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = GenericRobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass