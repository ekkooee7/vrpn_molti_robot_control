#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool, Int32
import time

class MissionCoordinator:
    def __init__(self):
        rospy.init_node('mission_coordinator')

        # 从参数服务器获取需要管理的机器人列表
        self.robot_names = rospy.get_param('~robot_names', ['gx0', 'gx1', 'gx2'])
        if not self.robot_names:
            rospy.logerr("No robot names specified!")
            return

        self.total_robots = len(self.robot_names)
        rospy.loginfo(f"Coordinator started. Managing {self.total_robots} robots: {self.robot_names}")

        # 状态变量
        self.current_waypoint_index = -1
        self.robots_finished_current_waypoint = set()

        # 为“前往下一个路点”指令创建一个发布者
        self.next_waypoint_pub = rospy.Publisher('/mission_control/next_waypoint', Int32, queue_size=10, latch=True)

        # 为每个机器人创建一个订阅者，以接收它们的完成状态
        for name in self.robot_names:
            topic = f"/{name}/waypoint_reached"
            rospy.Subscriber(topic, Bool, self.waypoint_reached_callback, callback_args=name)
        
        rospy.loginfo("Coordinator is ready. Starting mission in 3 seconds...")
        time.sleep(3)
        self.advance_to_next_waypoint()

    def waypoint_reached_callback(self, msg, robot_name):
        # 如果收到的是True消息，并且是当前任务点的回报
        if msg.data:
            if robot_name not in self.robots_finished_current_waypoint:
                self.robots_finished_current_waypoint.add(robot_name)
                rospy.loginfo(f"'{robot_name}' has reached waypoint #{self.current_waypoint_index + 1}. "
                              f"({len(self.robots_finished_current_waypoint)}/{self.total_robots} finished)")

            # 检查是否所有机器人都完成了当前路点
            if len(self.robots_finished_current_waypoint) == self.total_robots:
                rospy.loginfo(f"--- All robots have reached waypoint #{self.current_waypoint_index + 1}. Proceeding to next. ---")
                self.advance_to_next_waypoint()

    def advance_to_next_waypoint(self):
        # 重置已完成机器人集合
        self.robots_finished_current_waypoint.clear()
        
        # 增加任务点索引
        self.current_waypoint_index += 1
        
        rospy.loginfo(f"*** Issuing command: GO to waypoint #{self.current_waypoint_index + 1} ***")
        
        # 发布新的路点索引
        self.next_waypoint_pub.publish(Int32(data=self.current_waypoint_index))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        coordinator = MissionCoordinator()
        coordinator.run()
    except rospy.ROSInterruptException:
        pass