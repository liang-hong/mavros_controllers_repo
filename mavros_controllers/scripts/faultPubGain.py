#!/usr/bin/env python3

import rospy
from controller_msgs.msg import FaultInject
from std_msgs.msg import Header

def publish_faults():
    rospy.init_node('fault_publisher_node', anonymous=True)
    pub = rospy.Publisher('/geometric/faultinject', FaultInject, queue_size=10)
    fault_time = 2 # 设置故障持续时长/secend
    rate = rospy.Rate(1/fault_time)

    fault = FaultInject()
    fault.fault_occur = 0 # 预置故障未发生
    fault.fault_values = [0.8, 1.0, 1.0, 1.0] # 设置故障位置和偏差大小

    rate.sleep()
    rate.sleep()

    for countNum in range(2):
        # print("Count: ", countNum)
        if fault.fault_occur==0 :
            fault.fault_occur = 1   # 故障类型1
            rospy.loginfo("Fault will start and last %d secends", fault_time)
            # print("Publishing fault_occur: ", fault.fault_occur)
            pub.publish(fault)
        else :
            fault.fault_occur = 0
            rospy.loginfo("Fault end")
            # print("Publishing fault_occur: ", fault.fault_occur)
            pub.publish(fault)
        rate.sleep()


if __name__ == '__main__':
    try:
        publish_faults()
    except rospy.ROSInterruptException:
        pass
