import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import Hobot.GPIO as GPIO
import time
from math import pi, fabs
from filterpy.kalman import KalmanFilter
import numpy as np
import gluemaster.headfiles.gluemaster_config as gc

# class AdaptiveLowPassFilter:
#     def __init__(self, base_alpha=0.1, sensitivity=0.05):
#         self.base_alpha = base_alpha  # 基础滤波强度
#         self.sensitivity = sensitivity  # 速度变化敏感度
#         self.last_speed = 0

#     def update(self, new_speed):
#         # 计算速度变化率，调整 alpha
#         speed_diff = abs(new_speed - self.last_speed)
#         alpha = self.base_alpha + speed_diff * self.sensitivity
#         alpha = min(max(alpha, 0.05), 0.5)  # 限制 alpha 范围
#         filtered_speed = alpha * new_speed + (1 - alpha) * self.last_speed
#         self.last_speed = filtered_speed
#         return filtered_speed

class EncoderPublisher(Node):
    def __init__(self):
        super().__init__('encoder_publisher')
        
        self.phase_a = 29#pin
        self.phase_a_n = 37#pin
        self.phase_z = 31#pin
        self.PPR = 1024
        self.R = 30
        self.topic_name = 'belt_speed_topic'
        
        self.pulse_count = 0
        self.pulse_n_count = 0
        self.current_speed = 0.0
        self.last_count = 0
        self.avg_count = 0
        self.z_flag = 0
        self.temp = 0
        self.delta_count = 0
        self.circum = 2*pi*self.R
        self.filtered_speed = 0
        self.times = 0

        self.current_ls = []
        self.filtered_ls = []
        self.current_avg = None
        self.filtered_avg = None
        self.ls_num = 5
        
        GPIO.setwarnings(False)
        self.gpio_setup()
        GPIO.add_event_detect(self.phase_a, GPIO.BOTH, callback=self.a_pulse_count, bouncetime=1)
        GPIO.add_event_detect(self.phase_a_n, GPIO.BOTH, callback=self.a_n_pulse_count, bouncetime=1)
        GPIO.add_event_detect(self.phase_z, GPIO.RISING, callback=self.z_pulse_count, bouncetime=1)

        self.publisher_ = self.create_publisher(Float32, self.topic_name, qos_profile=gc.qos_profile)
        time_period = 1.000
        self.timer = self.create_timer(time_period, self.speed_cal_callback)

        # self.filter = AdaptiveLowPassFilter()

        self.kf = KalmanFilter(dim_x=1, dim_z=1)    # 状态量（速度）和观测量均为1维
        self.kf.x = np.array([0.0])                 # 初始状态（速度）
        self.kf.F = np.array([[1.0]])               # 状态转移矩阵
        self.kf.H = np.array([[1.0]])               # 观测矩阵
        self.kf.P = np.array([[100.0]])             # 初始协方差矩阵
        self.kf.R = np.array([[0.8]])               # 观测噪声方差，若传感器噪声大（如抖动严重），增大
        self.kf.Q = np.array([[0.001]])             # 过程噪声方差，若系统动态变化快，增大
            
    def gpio_setup(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.phase_a, GPIO.IN, pull_up_down=GPIO.HIGH)
        GPIO.setup(self.phase_a_n, GPIO.IN, pull_up_down=GPIO.HIGH)
        GPIO.setup(self.phase_z, GPIO.IN, pull_up_down=GPIO.HIGH)
    
    def a_pulse_count(self, channel):
        self.pulse_count += 1

    def a_n_pulse_count(self, channel):
        self.pulse_n_count += 1 
    
    def z_pulse_count(self, channel):
        self.z_flag += 1
        self.pulse_count = 0
        self.pulse_n_count = 0
        if self.temp == 0:
            self.temp = 1
            self.z_flag = 0

    def reset_kalman_filter(self):
        self.kf.x = np.array([0.0])
        self.kf.P = np.array([[100.0]])
        print("reset_kalman_filter")

    def check_kalman_health(self):
        # 检查协方差矩阵是否过大
        if self.kf.P[0,0] > 1000:
            # self.get_logger().warning('Kalman covariance too large, resetting...')
            return False
        return True

    def calculate_current_avg(self):
        if len(self.current_ls) < self.ls_num:
            self.current_ls.append(self.current_speed)
        elif len(self.current_ls) == self.ls_num:
            self.current_ls.pop(0)
            self.current_ls.append(self.current_speed)
            self.current_avg = sum(self.current_ls)/self.ls_num
        # return self.current_avg

    def calculate_filtered_avg(self):
        if len(self.filtered_ls) < self.ls_num:
            self.filtered_ls.append(self.filtered_speed)
        elif len(self.filtered_ls) == self.ls_num:
            self.filtered_ls.pop(0)
            self.filtered_ls.append(self.filtered_speed)
            self.filtered_avg = sum(self.filtered_ls)/self.ls_num
        # return self.filtered_avg
            
    def speed_cal_callback(self):
        if self.temp:
            self.times += 1
            self.avg_count = (self.pulse_count + self.pulse_n_count) / 2
            self.delta_count = (self.z_flag * 2 * self.PPR + self.avg_count - self.last_count) / 2
            if self.delta_count < 0:
                self.z_flag = 1
                self.delta_count = (self.z_flag * 2 * self.PPR + self.avg_count - self.last_count) / 2

            # self.get_logger().info(f'z_flag = {self.z_flag}')
            # self.get_logger().info(f'pulse_count = {self.pulse_count}')
            # self.get_logger().info(f'pulse_n_count = {self.pulse_n_count}')
            # self.get_logger().warning(f'avg_count = {self.avg_count}')
            # self.get_logger().info(f'last_count = {self.last_count}')
            # self.get_logger().warning(f'delta_count = {self.delta_count}')

            self.last_count = self.avg_count
            self.z_flag = 0
            self.current_speed = self.delta_count / self.PPR * self.circum

            # self.get_logger().error(f'speed = {self.current_speed}')
            # self.filtered_speed = self.filter.update(self.current_speed)
            # self.get_logger().error(f'filtered_speed = {self.filtered_speed}')
            
            if self.times != 1:
                if self.current_speed <= 50:
                    self.kf.predict()
                    self.kf.update(self.current_speed)
                    self.filtered_speed = self.kf.x[0]
                    self.calculate_current_avg()
                    self.calculate_filtered_avg()
                    if self.current_avg is not None and self.filtered_avg is not None:
                        if fabs(self.current_avg - self.filtered_avg)>1 or not self.check_kalman_health():
                            self.reset_kalman_filter()
                            self.kf.predict()
                            self.kf.update(self.current_speed)
                            self.filtered_speed = self.kf.x[0]
                            # self.get_logger().warning(f'current_avg = {self.current_avg}')
                            # self.get_logger().warning(f'filtered_avg = {self.filtered_avg}')
                else:
                    self.reset_kalman_filter()
                    self.filtered_speed = -1.00
                    
                msg = Float32()
                msg.data = self.filtered_speed
                self.publisher_.publish(msg)
                # self.get_logger().error(f'self.kf.P[0,0] = {self.kf.P[0,0]}')
                # self.get_logger().error(f'current_speed = {self.current_speed}')
                # self.get_logger().error(f'filtered_speed = {self.filtered_speed}')

def main(args=None):
    rclpy.init(args=args)
    encoder_publisher = EncoderPublisher()
    encoder_publisher.get_logger().info('encoder_publisher节点开始运行！')
    rclpy.spin(encoder_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()