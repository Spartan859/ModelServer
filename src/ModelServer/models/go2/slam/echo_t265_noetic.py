#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry  # 假设话题使用的是Odometry消息类型
import numpy as np

def callback(data):
    # 回调函数，当收到消息时被调用
    rospy.loginfo("收到里程计消息:")
    # 打印完整消息
    rospy.loginfo(data)
    
    # 如果你只想打印部分字段，可以单独提取
    # 例如打印位置信息
    rospy.loginfo("位置: x=%f, y=%f, z=%f", 
                 data.pose.pose.position.x,
                 data.pose.pose.position.y,
                 data.pose.pose.position.z)
    # 例如打印姿态信息
    rospy.loginfo("姿态: x=%f, y=%f, z=%f, w=%f",
                 data.pose.pose.orientation.x,
                 data.pose.pose.orientation.y,
                 data.pose.pose.orientation.z,
                 data.pose.pose.orientation.w)
    
    position = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
    orientation = np.array([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    return position, orientation

def listener():
    # 初始化节点
    rospy.init_node('odom_printer', anonymous=True)
    
    # 订阅话题，第一个参数是话题名，第二个是消息类型，第三个是回调函数
    rospy.Subscriber("/camera/odom/sample", Odometry, callback)
    
    # 保持节点运行，直到被终止
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
