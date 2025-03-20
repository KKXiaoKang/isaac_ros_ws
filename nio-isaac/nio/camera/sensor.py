# from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera, IMUSensor
import numpy as np

import rospy
from std_msgs.msg import Float32
"""
    imu_data:  
    { 'time': 0.27000001072883606, 
      'physics_step': 111.0, 
      'lin_acc': array([ 0.22668934, -0.01075992,  9.798907  ], dtype=float32), 
      'ang_vel': array([-0.0045824 , -0.03133204, -0.00079955], dtype=float32), 
      'orientation': array([ 9.9981683e-01,  4.3590643e-04, -1.9111114e-02,  9.3401712e-04], dtype=float32)}
"""

class Sensor:
    def __init__(self, usd_path, prim_path):  # pylint: disable=W0613
        # add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
        self.color_sensor = Camera(f"{prim_path}/RSD455/Camera_OmniVision_OV9782_Color")
        self.depth_sensor = Camera(f"{prim_path}/RSD455/Camera_Pseudo_Depth")
        # 创建IMU传感器，设置采样频率和滤波器大小
        self.imu_sensor = IMUSensor(
            prim_path=f"{prim_path}/base_link/Imu_Sensor",
            name="imu_sensor",
            frequency=1000,  # 设置采样频率为1000Hz
            # 设置滤波器大小
            linear_acceleration_filter_size=10,
            angular_velocity_filter_size=5,
            orientation_filter_size=5
        )
        # 创建ros发布器
        self.freq_pub = rospy.Publisher('/kuavo_isaac_sim/sensor_estimate', Float32, queue_size=10)
    
    def initialize(self):
        self.color_sensor.initialize()
        self.depth_sensor.initialize()
        self.imu_sensor.initialize()

    def get_obs(self):
        # d455相机图像 
        self.color_sensor.get_current_frame()
        self.depth_sensor.get_current_frame()
        
        # 获取IMU数据
        imu_data = self.imu_sensor.get_current_frame(read_gravity=True) # 读取带重力的IMU数据
        
        # 统计频率
        float_data = Float32()
        float_data.data = 1.0
        self.freq_pub.publish(float_data)
        
        data = {
            "rgb": self.color_sensor.get_rgb(),
            "depth": self.depth_sensor.get_depth(),
            "imu": {
                "time": imu_data["time"],                # 时间戳
                "physics_step": imu_data["physics_step"], # 物理仿真步数
                "linear_acceleration": imu_data["lin_acc"],  # 线性加速度（带重力）
                "angular_velocity": imu_data["ang_vel"],    # 角速度
                "orientation": imu_data["orientation"]      # 方向四元数
            }
        }
        return data
