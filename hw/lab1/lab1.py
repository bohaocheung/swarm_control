#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget
import math


class FormationController:
    def __init__(self, uav_list):
        """
        初始化 FormationController，并订阅/发布 MAVROS 话题
        :param uav_list: 无人机名称列表，例如 ["uav1", "uav2", "uav3", "uav4", "uav5"]
        """
        self.uav_list = uav_list

        """添加期望位置和速度函数"""
        self.timesteps = 0
        self.r = 7
        self.w = 0.214

        # 这个示例代码中是固定编队，所以位置偏移向量不为0，速度偏移向量是0，不需要编队补偿控制输入，因为速度偏移向量求导是0.
        # 如果需要扩展到时变编队，则需要根据当前时刻（可以通过ros的函数来获得），计算实时的位置与速度偏移向量。同时需要计算编队补偿
        self.data = {
            "uav1": {"pose": None, "velocity": None, "hpx": 0.0, "hpy": 1.5, "hvx": 0.0, "hvy": 0.0},
            "uav2": {"pose": None, "velocity": None, "hpx": 1.0, "hpy": -1.0, "hvx": 0.0, "hvy": 0.0},
            "uav3": {"pose": None, "velocity": None, "hpx": -1.0, "hpy": -1.0, "hvx": 0.0, "hvy": 0.0},
            "uav4": {"pose": None, "velocity": None, "hpx": -1.0, "hpy": -1.0, "hvx": 0.0, "hvy": 0.0},
            "uav5": {"pose": None, "velocity": None, "hpx": -1.0, "hpy": -1.0, "hvx": 0.0, "hvy": 0.0}
        }
        self.cmd_publishers = {}
        self.error_publishers = {}

        for i, uav in enumerate(self.uav_list):
            # 订阅位姿和速度话题
            rospy.Subscriber(f'/{uav}/mavros/local_position/pose', PoseStamped,
                             lambda msg, idx=i: self.pose_callback(msg, idx))
            rospy.Subscriber(f'/{uav}/mavros/local_position/velocity_local', TwistStamped,
                             lambda msg, idx=i: self.velocity_callback(msg, idx))
            
            # 设置控制话题发布者
            self.cmd_publishers[uav] = rospy.Publisher(f'/{uav}/mavros/setpoint_raw/local', 
                                                       PositionTarget, queue_size=10)
            self.error_publishers[uav] = rospy.Publisher(f'/{uav}/alg/error', 
                                                       TwistStamped, queue_size=10)

        # 定时器，每 1/20 秒（50ms）调用一次 publish_control_commands
        self.timer = rospy.Timer(rospy.Duration(1.0 / 20), self.publish_control_commands)



    def pose_callback(self, msg, idx):
        """ 处理位姿数据并存储 """
        uav_name = self.uav_list[idx]
        self.data[uav_name]["pose"] = msg

    def velocity_callback(self, msg, idx):
        """ 处理速度数据并存储 """
        uav_name = self.uav_list[idx]
        self.data[uav_name]["velocity"] = msg


    def g(self, t, i):
        x = math.sin(self.w*t/2 + math.pi*i/5)
        if x > 0:
            return 1
        if x < 0:
            return -1
        return 0


    def publish_control_commands(self, event=None):
        """ 计算当前时间 """
        time = self.timesteps * 0.05
        self.timesteps += 1

        """ 用于计算误差 """
        center_px, center_py, center_vx, center_vy = 0.0, 0.0, 0.0, 0.0
        for uav in self.uav_list:
            center_px += (self.data[uav]["pose"].pose.position.x - self.data[uav]["hpx"])/len(self.uav_list)
            center_py += (self.data[uav]["pose"].pose.position.y - self.data[uav]["hpy"])/len(self.uav_list)
            center_vx += (self.data[uav]["velocity"].twist.linear.x - self.data[uav]["hvx"])/len(self.uav_list)
            center_vy += (self.data[uav]["velocity"].twist.linear.y - self.data[uav]["hvy"])/len(self.uav_list)

        for i, uav in enumerate(self.uav_list):
            """ 发布控制指令（仅使用 x/y 加速度和 z 位置，偏航角固定为 0) """
            if self.data[uav]["pose"] is None or self.data[uav]["velocity"] is None:
                continue
            
            """ 当前无人机的位置和速度 """
            pos_uav = self.data[uav]["pose"].pose.position
            vel_uav = self.data[uav]["velocity"].twist.linear
            
            """ 当前时刻期望的无人机的位置和速度 """
            self.data[uav]["hpx"] = self.r*(math.cos(self.w*time + 2*math.pi*i/5) - 1)*self.g(time, i)
            self.data[uav]["hpy"] = self.r*math.sin(self.w*time + 2*math.pi*i/5)
            self.data[uav]["hvx"] = -self.w*self.r*math.sin(self.w*time + 2*math.pi*i/5)*self.g(time, i)
            self.data[uav]["hvy"] = self.w*self.r*math.cos(self.w*time + 2*math.pi*i/5)

            """ K1 和 K2 默认沿用示例代码参数"""
            acc_x = -2 * (pos_uav.x - self.data[uav]["hpx"]) -1.2 * (vel_uav.x - self.data[uav]["hvx"])
            acc_y = -2 * (pos_uav.y - self.data[uav]["hpy"]) -1.2 * (vel_uav.y - self.data[uav]["hvy"])
            
            """ 计算时变编队补偿控制输入 """
            acc_x += -self.w**2*self.r*math.cos(self.w*time+2*math.pi*i/5)*self.g(time, i)
            acc_y += -self.w**2*self.r*math.sin(self.w*time+2*math.pi*i/5)

            """ 计算其他节点信息 """
            other = i
            if other == 0:
                other = 5
            other_uav = "uav" + str(other)

            if self.data[other_uav]["pose"] is None or self.data[other_uav]["velocity"] is None:
                continue
                
            pos_other = self.data[other_uav]["pose"].pose.position
            vel_other = self.data[other_uav]["velocity"].twist.linear
            
            acc_x += 0.3416 * (pos_other.x - self.data[other_uav]["hpx"] - pos_uav.x + self.data[uav]["hpx"]) + \
                        0.7330 * (vel_other.x - self.data[other_uav]["hvx"] - vel_uav.x + self.data[uav]["hvx"])
            acc_y += 0.3416 * (pos_other.y - self.data[other_uav]["hpy"] - pos_uav.y + self.data[uav]["hpy"]) + \
                        0.7330 * (vel_other.y - self.data[other_uav]["hvy"] - vel_uav.y + self.data[uav]["hvy"])

            cmd_msg = PositionTarget()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.coordinate_frame = 1

            # 设置控制模式：使用 x/y 加速度，z 位置，yaw 角度固定
            cmd_msg.type_mask = (
                PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |  # 忽略 x/y 位置
                PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |  # 忽略速度
                PositionTarget.IGNORE_AFZ |  # 忽略 z 方向加速度
                PositionTarget.IGNORE_YAW_RATE  # 忽略 yaw 速率
            )

            # 控制器
            cmd_msg.position.z = 1.0  # 例如，目标高度 1m
            cmd_msg.yaw = 0.0  # 固定 yaw 角度
            
            cmd_msg.acceleration_or_force.x = acc_x
            cmd_msg.acceleration_or_force.y = acc_y

            # 发布控制命令
            self.cmd_publishers[uav].publish(cmd_msg)

            """ 发布控制误差 """
            error_msg = TwistStamped()
            error_msg.twist.linear.x = pos_uav.x - self.data[uav]["hpx"] - center_px # x轴位置误差
            error_msg.twist.linear.y = pos_uav.y - self.data[uav]["hpy"] - center_py # y轴位置误差
            error_msg.twist.angular.x = vel_uav.x - self.data[uav]["hvx"]- center_vx # x轴速度误差
            error_msg.twist.angular.y = vel_uav.y - self.data[uav]["hvy"]- center_vy # y轴速度误差

            self.error_publishers[uav].publish(error_msg)

def main():
    rospy.init_node('formation_controller_node', anonymous=True)

    # 设定要监听和控制的无人机
    uav_list = ["uav1", "uav2", "uav3", "uav4", "uav5"]
    
    # 初始化控制器（自动启动定时器）
    controller = FormationController(uav_list)

    rospy.spin()  # 让 ROS 节点保持运行

if __name__ == '__main__':
    main()
