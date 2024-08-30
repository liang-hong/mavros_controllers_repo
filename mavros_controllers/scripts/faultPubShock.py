#!/usr/bin/env python3

import rospy
from controller_msgs.msg import FaultInject
from std_msgs.msg import Header

def publish_faults():
    rospy.init_node('fault_publisher_node', anonymous=True)
    pub = rospy.Publisher('/geometric/faultinject', FaultInject, queue_size=10)
    rate = rospy.Rate(8) #设置振荡频率/Hz
    fault_time = 2 # 设置故障持续时长/secend

    fault = FaultInject()
    fault.fault_occur = 0   # 预置未发生故障
    fault.fault_values = [1.0, 1.0, 1.0, 1.0] # occur=0，偏差不起作用

    faultp = FaultInject()
    faultp.fault_occur = 2  # 故障类型2
    faultp.fault_values = [1.2, 1.0, 1.0, 1.0] # 配置正向偏差状况

    faultn = FaultInject()
    faultn.fault_occur = 2  # 故障类型2
    faultn.fault_values = [0.8, 1.0, 1.0, 1.0] # 配置反向偏差状况

    rate.sleep()
    rate.sleep()
    countNum = 0
    start_time = rospy.get_rostime()
    rospy.loginfo("Fault will start and last %d secends", fault_time)
    end_time = start_time + rospy.Duration.from_sec(fault_time)
    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
    # while not rospy.is_shutdown() :
        # print("Count: ", countNum)
        if countNum%2 :
            # print("Publishing fault_occur: ", faultp.fault_occur)
            pub.publish(faultp)
        else:
            # print("Publishing fault_occur: ", faultn.fault_occur)
            pub.publish(faultn)
        countNum += 1
        rate.sleep()

    # print("Count: ", countNum)
    # print("Publishing fault_occur: ", fault.fault_occur)
    rospy.loginfo("Fault end")
    pub.publish(fault)
    rate.sleep()


if __name__ == '__main__':
    try:
        publish_faults()
    except rospy.ROSInterruptException:
        pass
