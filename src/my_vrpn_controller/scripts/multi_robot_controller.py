#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import PoseStamped, Twist, Pose
from tf.transformations import euler_from_quaternion

class GenericRobotController:
    def __init__(self):
        rospy.init_node('generic_robot_controller', anonymous=True)

        self.robot_name = rospy.get_param('~robot_name', 'default_robot')
        if self.robot_name == 'default_robot':
            rospy.logwarn("robot_name parameter not set! Using 'default_robot'.")

        vrpn_topic_name = f"/vrpn_client_node/{self.robot_name}/pose"
        cmd_vel_topic_name = f"/{self.robot_name}/cmd_vel"

        waypoints_param = rospy.get_param('~waypoints', [[0.0, 0.0]])
        self.waypoints = [tuple(p) for p in waypoints_param]
        
        self.waypoint_index = 0
        self.goal_x = self.waypoints[self.waypoint_index][0]
        self.goal_y = self.waypoints[self.waypoint_index][1]

        # --- 控制器参数 ---
        # 使用您之前验证过的、可行的单机器人参数作为默认值
        self.distance_tolerance = rospy.get_param('~distance_tolerance', 400.0)
        self.angle_tolerance = rospy.get_param('~angle_tolerance', 0.2)
        self.linear_speed_gain = rospy.get_param('~linear_speed_gain', 0.2)
        self.angular_speed_gain = rospy.get_param('~angular_speed_gain', 1.0)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 800.0) # mm/s
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.8) # rad/s
        self.yaw_offset = rospy.get_param('~yaw_offset', 0.0)

        # --- 状态变量 ---
        # <<< MODIFIED: 不再需要 self.current_pose 变量 >>>
        self.all_goals_reached = False

        # --- ROS发布者和订阅者 ---
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic_name, Twist, queue_size=10)
        self.vrpn_sub = rospy.Subscriber(vrpn_topic_name, PoseStamped, self.pose_callback)

        rospy.loginfo(f"[{self.robot_name}] Controller started. Yaw offset: {self.yaw_offset:.3f} rad.")
        self.log_current_goal()

    def log_current_goal(self):
        rospy.loginfo(f"[{self.robot_name}] Heading to waypoint #{self.waypoint_index + 1}: x={self.goal_x:.2f}, y={self.goal_y:.2f} (mm)")

    def pose_callback(self, msg):
        """
        当接收到新的位姿信息时，此回调函数被调用
        """
        # <<< MODIFIED: 直接将收到的 msg.pose 作为参数传递，而不是设置一个共享变量 >>>
        if not self.all_goals_reached:
            self.control_loop(msg.pose)

    def control_loop(self, current_pose): # <<< MODIFIED: 接受一个 pose 对象作为参数
        """
        核心控制逻辑
        """
        # <<< MODIFIED: 不再需要检查 self.current_pose 是否为 None >>>
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

        # --- 后续逻辑与之前完全相同 ---
        if distance_to_goal < self.distance_tolerance:
            rospy.loginfo(f"[{self.robot_name}] Waypoint #{self.waypoint_index + 1} reached!")
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.waypoints):
                self.all_goals_reached = True
                rospy.loginfo(f"[{self.robot_name}] All waypoints reached! Mission complete.")
            else:
                self.goal_x = self.waypoints[self.waypoint_index][0]
                self.goal_y = self.waypoints[self.waypoint_index][1]
                self.log_current_goal()
        else:
            if abs(angle_error) > self.angle_tolerance:
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = self.angular_speed_gain * angle_error
            else:
                expected_speed_mms = self.linear_speed_gain * distance_to_goal
                cmd_vel_msg.linear.x = expected_speed_mms / 1000.0
                cmd_vel_msg.angular.z = self.angular_speed_gain * angle_error
        
        if self.all_goals_reached:
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0

        max_linear_speed_ms = self.max_linear_speed / 1000.0
        cmd_vel_msg.linear.x = min(cmd_vel_msg.linear.x, max_linear_speed_ms)
        cmd_vel_msg.angular.z = max(min(cmd_vel_msg.angular.z, self.max_angular_speed), -self.max_angular_speed)

        # 最终发布指令
        self.cmd_vel_pub.publish(cmd_vel_msg)
        
        if not self.all_goals_reached:
            rospy.loginfo_throttle(1, f"[{self.robot_name}] Dist:{distance_to_goal:.2f} AngErr:{angle_error:.2f} | Lin.X:{cmd_vel_msg.linear.x:.3f}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = GenericRobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass