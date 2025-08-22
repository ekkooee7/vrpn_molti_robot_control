#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion

class VRPNtoCmdVelController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('vrpn_to_cmdvel_controller', anonymous=True)

        # --- 参数 ---
        # VRPN Tracker的话题名称
        vrpn_topic_name = rospy.get_param('~vrpn_topic', '/vrpn_client_node/gx2/pose')

        # --- 路径点 (单位: 毫米) ---
        # 坐标单位为毫米 (mm)
        self.waypoints = [
            (27.0 , 5426.0),
            (-1333.0 , 4307.0),
            (974.0 , 2616.0)
        ]
        self.waypoint_index = 0
        self.goal_x = self.waypoints[self.waypoint_index][0]
        self.goal_y = self.waypoints[self.waypoint_index][1]

        # --- 控制器参数 (全部统一为毫米/秒体系) ---
        # <<< FIX: 将所有参数调整到毫米尺度，并设定合理的值 >>>
        self.distance_tolerance = 400.0  # 容忍度: 300毫米 (30厘米)，这是一个合理的值
        self.angle_tolerance = 0.2       # 角度容忍度: 0.2弧度 (约11.5度)，比1.0更精确
        self.linear_speed_gain = 0.2     # 线性速度增益: 调小以适应毫米单位的大误差值
        self.angular_speed_gain = 1.0    # 角速度增益: 1.0可以提供较快的转向响应
        self.max_linear_speed = 800.0   # 最大线速度: 1000毫米/秒 (等于1.0米/秒)
        self.max_angular_speed = 0.8     # 最大角速度: 0.8弧度/秒

        # --- 状态变量 ---
        self.current_pose = None
        self.all_goals_reached = False

        # --- ROS发布者和订阅者 ---
        # 确保发布的Topic名称正确
        self.cmd_vel_pub = rospy.Publisher('/gx2/cmd_vel', Twist, queue_size=10)
        self.vrpn_sub = rospy.Subscriber(vrpn_topic_name, PoseStamped, self.pose_callback)

        rospy.loginfo("Controller started in MILLIMETER mode for tracker 'gx2'.")
        self.log_current_goal()

    def log_current_goal(self):
        """打印当前的目标点信息"""
        rospy.loginfo("Heading to waypoint #%d: x=%.2f, y=%.2f (mm)",
                      self.waypoint_index + 1, self.goal_x, self.goal_y)

    def pose_callback(self, msg):
        """当接收到新的位姿信息时，此回调函数被调用"""
        self.current_pose = msg.pose
        if not self.all_goals_reached:
            self.control_loop()

    def control_loop(self):
        """核心控制逻辑"""
        if self.current_pose is None:
            rospy.logwarn_throttle(2, "Waiting for pose data from VRPN...")
            return

        # current_x, current_y 单位是毫米 (mm)
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        orientation_q = self.current_pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        
        # distance_to_goal 单位是毫米 (mm)
        distance_to_goal = math.sqrt((self.goal_x - current_x)**2 + (self.goal_y - current_y)**2)
        angle_to_goal = math.atan2(self.goal_y - current_y, self.goal_x - current_x)
        
        angle_error = angle_to_goal - yaw
        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2 * math.pi

        cmd_vel_msg = Twist()

        if distance_to_goal < self.distance_tolerance:
            rospy.loginfo("Waypoint #%d reached!", self.waypoint_index + 1)
            self.waypoint_index += 1
            
            if self.waypoint_index >= len(self.waypoints):
                self.all_goals_reached = True
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
                rospy.loginfo("All waypoints reached! Mission complete.")
            else:
                self.goal_x = self.waypoints[self.waypoint_index][0]
                self.goal_y = self.waypoints[self.waypoint_index][1]
                self.log_current_goal()
        else:
            if abs(angle_error) > self.angle_tolerance:
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = self.angular_speed_gain * angle_error
            else:
                # 期望速度，单位是 毫米/秒 (mm/s)
                expected_speed_mms = self.linear_speed_gain * distance_to_goal
                
                # <<< FIX: 将计算出的速度从 毫米/秒(mm/s) 转换为 米/秒(m/s) 以便发布 >>>
                # 因为ROS机器人驱动通常期望 /cmd_vel 的单位是 米/秒
                cmd_vel_msg.linear.x = expected_speed_mms / 1000.0
                cmd_vel_msg.angular.z = self.angular_speed_gain * angle_error
        
        # <<< FIX: 速度限制，单位统一为 米/秒 和 弧度/秒 >>>
        # 将毫米/秒的最大速度转换为米/秒以进行比较
        max_linear_speed_ms = self.max_linear_speed / 1000.0
        cmd_vel_msg.linear.x = min(cmd_vel_msg.linear.x, max_linear_speed_ms)
        
        # 角速度单位是 弧度/秒，不需要变
        cmd_vel_msg.angular.z = max(min(cmd_vel_msg.angular.z, self.max_angular_speed), -self.max_angular_speed)

        # 发布最终的Twist指令 (linear.x 单位是 m/s)
        self.cmd_vel_pub.publish(cmd_vel_msg)

        if not self.all_goals_reached:
            rospy.loginfo_throttle(1, "Dist:%.2f(mm), AngErr:%.2f | Lin.X:%.3f(m/s), Ang.Z:%.2f(rad/s)",
                                   distance_to_goal, angle_error,
                                   cmd_vel_msg.linear.x, cmd_vel_msg.angular.z)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = VRPNtoCmdVelController()
        controller.run()
    except rospy.ROSInterruptException:
        pass