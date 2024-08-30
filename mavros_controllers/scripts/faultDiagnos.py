import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from controller_msgs.msg import FaultInject
import numpy as np
import statistics
from collections import deque

import matplotlib.pyplot as plt
import scipy.io as sio
import os

class FaultDiagnos:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('fault_observer', anonymous=True)
        
        # 订阅电机推力话题
        self.motor_thrust_sub = rospy.Subscriber('/geometric/motorthrust', Float64MultiArray, self.motor_thrust_callback, queue_size=1)
        #订阅无人机状态
        self.velocity_sub = rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.velocity_callback, queue_size=1)
        # 订阅故障类型用于收集数据和验证
        self.fault_sub = rospy.Subscriber('/geometric/faultinject', FaultInject, self.fault_callback, queue_size=1)

        # 初始化标志
        self.new_motor_thrust_available = False
        self.new_velocity_available = False
        
        # 故障观测器的状态变量
        self.J = np.array([[0.029125, 0, 0], 
                           [0, 0.029125, 0], 
                           [0, 0, 0.055225]])
        Mix = np.array([[-1/np.sqrt(2),  1/np.sqrt(2),  1, 1], 
                             [ 1/np.sqrt(2), -1/np.sqrt(2),  1, 1], 
                             [ 1/np.sqrt(2),  1/np.sqrt(2), -1, 1], 
                             [-1/np.sqrt(2), -1/np.sqrt(2), -1, 1]])
        Mix_inv = np.linalg.inv(Mix)
        self.Mix_invRot = Mix_inv[:3, :]   # 混空分配的逆运算, 同时也是输入矩阵

        self.Omega_hat = np.zeros(3)  # 观测器状态估计
        self.A = np.array([[-2, 0, 0], 
                           [0, -2, 0], 
                           [0, 0, -2]])  # 系统矩阵

        error_length = 20 # 误差队列长度
        self.error_queue = deque([0.0 for _ in range(error_length)], maxlen=error_length)
        self.stati_length = 10 # 诊断队列长度
        self.mean_queue = deque([0.0 for _ in range(self.stati_length)], maxlen=self.stati_length)
        self.vari_queue = deque([0.0 for _ in range(self.stati_length)], maxlen=self.stati_length)
        samplefreq = 100 # 采样频率Hz
        self.dt = 1/samplefreq
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)

        statinum = 10 # 诊断运算间隔
        self.statistics_timer = rospy.Timer(rospy.Duration(self.dt * statinum), self.statistics_timer_callback)
        self.faultstate = 0 # 故障类型真实值
        self.faultdiag = 0  # 故障类型诊断值

        # 收集数据到diag_data.mat文件, 初始化
        if os.path.exists('diag_data.mat'):
            # 删除文件
            os.remove('diag_data.mat')
            rospy.loginfo(f"File '{'diag_data.mat'}' has been deleted.")
        else:
            rospy.loginfo(f"File '{'diag_data.mat'}' does not exist.")


    def motor_thrust_callback(self, motor_thrust):
        # 每当有新消息到达时，将其存储在latest_msg中，并设置new_msg_available标志
        self.latest_motor_thrust = motor_thrust
        self.new_motor_thrust_available = True

    def velocity_callback(self, velocity):
        self.latest_velocity = velocity
        self.new_velocity_available = True

    def fault_callback(self, fault):
        self.faultstate = fault.fault_occur

    def timer_callback(self, event):
        if self.new_motor_thrust_available and self.new_velocity_available :
            # 如果有新消息
            motor_thrust = np.array(self.latest_motor_thrust.data)
            # 获取当前的实际输出
            Omega = np.array([ self.latest_velocity.twist.angular.x, 
                              -self.latest_velocity.twist.angular.y, 
                              -self.latest_velocity.twist.angular.z])
            # 更新故障观测器
            error = Omega - self.Omega_hat
            # print(np.linalg.norm(error))
            self.error_queue.append(np.linalg.norm(error))
            Omega_hat_dot = -np.dot(self.A, error) - np.dot(np.linalg.inv(self.J), np.cross(Omega, np.dot(self.J, Omega))) + np.dot(np.linalg.inv(self.J), np.dot(self.Mix_invRot, motor_thrust))
            self.Omega_hat += Omega_hat_dot * self.dt
            # 清除标志，表示已处理最新消息
            self.new_motor_thrust_available = False
            self.new_velocity_available = False
        
    def statistics_timer_callback(self, event):
         if self.error_queue:
            # 计算队列中的均值
            mean = statistics.mean(self.error_queue)
            # 计算队列中的方差
            variance = statistics.variance(self.error_queue)
            # print(f"Mean: {mean:.6f}, Variance: {variance:.6f}")

            self.mean_queue.append(mean)    
            mean_mean = statistics.mean(self.mean_queue)
            self.vari_queue.append(5.0*variance)
            vari_mean = statistics.mean(self.vari_queue)
            if mean_mean > 0.7 and self.faultdiag==0:
                if vari_mean < 0.25 :
                    self.faultdiag = 1
                    rospy.loginfo("Gain fault occur!")
                elif vari_mean > 0.8 :
                    self.faultdiag = 2
                    rospy.loginfo("Shock fault occur!")
            elif mean_mean < 0.4 and vari_mean < 0.15 :
                if self.faultdiag != 0 :
                    rospy.loginfo("No fault")
                self.faultdiag = 0

            # 收集数据到diag_data.mat文件
            self.append_data_to_mat([self.get_ros_time(), mean, 5.0*variance, mean_mean, vari_mean, self.faultstate, self.faultdiag], 'diag_data.mat', 'row')

            # # 清除之前的图表
            # plt.cla()
            # # 绘制新的数据
            # plt.plot(np.arange(0.0, 0.1*self.stati_length, 0.1), self.mean_queue, label='mean')
            # plt.plot(np.arange(0.0, 0.1*self.stati_length, 0.1), self.vari_queue, label='variance')
            # plt.ylim(0, 2)
            # plt.legend(loc='upper left')
            # plt.title('Live Data Plot')
            # plt.xlabel('Time(s)')
            # plt.ylabel('Value')
            # # 显示图表
            # plt.pause(0.001)  # 必须有一个短暂停顿，以便窗口刷新
        
         else:
            print("No data in the queue.")


    def get_ros_time(self):
        # 获取当前的 ROS 时间
        current_time = rospy.get_rostime()
        # 将 ROS 时间转换为以秒为单位的浮点数
        time_in_seconds = current_time.to_sec()
        # 返回转换后的浮点数时间
        return time_in_seconds
    

    def append_data_to_mat(self, data, filename, mode):
        """
        将数据追加到现有的 `.mat` 文件中。
        参数:
        - data: 要追加的数据
        - filename: `.mat` 文件名
        - mode: 追加模式，'row' 表示追加到最后一行，'col' 表示追加到最后一列
        """
        # 检查文件是否存在
        if not os.path.exists(filename):
            # 如果文件不存在，则创建一个新的空矩阵
            existing_data = np.empty((0, len(data)))
        else:
            # 如果文件存在，加载现有数据
            loaded_data = sio.loadmat(filename)
            # 获取数据，如果没有 'data' 键则创建一个新的空矩阵
            existing_data = loaded_data.get('data', np.empty((0, len(data))))

        if mode == 'row':
            # 如果是追加到最后一行
            new_data = np.vstack((existing_data, data))
        elif mode == 'col':
            # 如果是追加到最后一列
            new_data = np.hstack((existing_data, data.reshape(-1, 1)))

        # 保存新数据
        sio.savemat(filename, {'data': new_data})

if __name__ == '__main__':
    try:
        fo = FaultDiagnos()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass