#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

def move_forward_test():
    """
    一个简单的测试脚本，持续向指定机器人发送缓慢前进的指令。
    """
    # 初始化一个临时的测试节点
    rospy.init_node('robot_move_tester', anonymous=True)

    # 从ROS参数服务器获取要控制的机器人名称，默认为'gx1'
    # 这样我们就可以用一个脚本测试所有机器人
    robot_name = rospy.get_param('~robot_name', 'gx1')

    # 根据机器人名称，动态构建要发布的话题名
    cmd_vel_topic = f"/{robot_name}/cmd_vel"

    # 创建一个发布者，发布到目标话题
    pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    # 设置发布频率为10Hz (这是/cmd_vel的标准频率)
    rate = rospy.Rate(10)

    # 创建一个Twist消息实例
    twist_msg = Twist()

    # 设置缓慢前进的速度
    # linear.x > 0 代表前进
    # 我们用0.1米/秒作为一个安全、缓慢的速度
    slow_forward_speed = 0.1
    twist_msg.linear.x = slow_forward_speed
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0
    twist_msg.angular.x = 0.0
    twist_msg.angular.y = 0.0
    twist_msg.angular.z = 0.0 # angular.z = 0 代表不转弯

    rospy.loginfo(f"测试开始：持续向话题 '{cmd_vel_topic}' 发送前进指令 (速度: {slow_forward_speed} m/s)...")
    rospy.loginfo("按 Ctrl+C 停止测试。")

    # 循环发布，直到节点被关闭 (例如按了Ctrl+C)
    while not rospy.is_shutdown():
        # 发布消息
        pub.publish(twist_msg)
        
        # 按照设定的频率休眠
        rate.sleep()

    rospy.loginfo("测试结束。发送停止指令。")
    # 在退出前发送一次停止指令，确保机器人停下
    twist_msg.linear.x = 0.0
    pub.publish(twist_msg)

if __name__ == '__main__':
    try:
        move_forward_test()
    except rospy.ROSInterruptException:
        pass