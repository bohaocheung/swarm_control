#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget
import math

class FormationTrackController:
    def __init__(self, uav_list):
        """
        初始化 FormationController，并订阅/发布 MAVROS 话题
        :param uav_list: 无人机名称列表，例如 ["uav1", "uav2", "uav3", "uav4", "uav5"]
        """
        self.uav_list = uav_list
        self.r = 2
        self.w = 0.05
        
        # 这个示例代码中是固定编队，所以位置偏移向量不为0，速度偏移向量是0，不需要编队补偿控制输入，因为速度偏移向量求导是0.
        # 如果需要扩展到时变编队，则需要根据当前时刻（可以通过ros的函数来获得），计算实时的位置与速度偏移向量。同时需要计算编队补偿控制输入。
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

        # 记录启动时间
        self.start_time = rospy.Time.now()

    def pose_callback(self, msg, idx):
        """ 处理位姿数据并存储 """
        uav_name = self.uav_list[idx]
        self.data[uav_name]["pose"] = msg

    def velocity_callback(self, msg, idx):
        """ 处理速度数据并存储 """
        uav_name = self.uav_list[idx]
        self.data[uav_name]["velocity"] = msg

    def publish_control_commands(self, event=None):

        # 计算已经运行的时间
        elapsed_time = (rospy.Time.now() - self.start_time).to_sec()

        """ 计算领导者信息 """
        center_px = 5 * math.cos(0.1 * elapsed_time)
        center_py = 5 * math.sin(0.1 * elapsed_time)
        center_vx = -0.5 * math.sin(0.1 * elapsed_time)
        center_vy = 0.5 * math.cos(0.1 * elapsed_time)

        for i,uav in enumerate(self.uav_list):
            """ 发布控制指令（仅使用 x/y 加速度和 z 位置，偏航角固定为 0） """
            if self.data[uav]["pose"] is None or self.data[uav]["velocity"] is None:
                continue
            
            pos_uav = self.data[uav]["pose"].pose.position
            vel_uav = self.data[uav]["velocity"].twist.linear
            
            """ 计算状态偏移向量 """
            self.data[uav]["hpx"] = self.r * math.cos(self.w * elapsed_time + 2 * math.pi * i / 5)
            self.data[uav]["hpy"] = self.r * math.sin(self.w * elapsed_time + 2 * math.pi * i / 5)
            self.data[uav]["hvx"] = -self.w * self.r * math.sin(self.w * elapsed_time + 2 * math.pi * i / 5)
            self.data[uav]["hvy"] = self.w *self.r * math.cos(self.w * elapsed_time + 2 * math.pi * i / 5)

            """ 计算自身误差 """
            delta_x = - 0.3416 * (pos_uav.x - self.data[uav]["hpx"] - center_px) - 0.7330 * (vel_uav.x - self.data[uav]["hvx"] - center_vx)
            delta_y = - 0.3416 * (pos_uav.y - self.data[uav]["hpy"] - center_py) - 0.7330 * (vel_uav.y - self.data[uav]["hvy"] - center_vy)
            
            """ 计算其他节点信息 """
            left = i + 2
            right = i 
            if left == 6:
                left = 1
            if right == 0:
                right = 5
            neighbors = ["uav" + str(i) for i in (left, right)]

            for other_uav in neighbors:
                if other_uav == uav:
                    continue
                
                if self.data[other_uav]["pose"] is None or self.data[other_uav]["velocity"] is None:
                    continue
                
                pos_other = self.data[other_uav]["pose"].pose.position
                vel_other = self.data[other_uav]["velocity"].twist.linear
                
                delta_x += 0.3416 * (pos_other.x - self.data[other_uav]["hpx"] - pos_uav.x + self.data[uav]["hpx"]) + \
                         0.7330 * (vel_other.x - self.data[other_uav]["hvx"] - vel_uav.x + self.data[uav]["hvx"])
                delta_y += 0.3416 * (pos_other.y - self.data[other_uav]["hpy"] - pos_uav.y + self.data[uav]["hpy"]) + \
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
            
            c = 1.0
            b = 0.1 
            sigma = 0.01
            phi = c + b / (math.sqrt(delta_x**2 + delta_y**2) + sigma)

            acc_x = phi * delta_x
            acc_y = phi * delta_y

            """ 计算时变编队补偿控制输入 """
            acc_x += self.w**2 * self.r * math.cos(self.w * elapsed_time + 2*math.pi*i/5)
            acc_y += self.w**2 * self.r * math.sin(self.w * elapsed_time + 2*math.pi*i/5)

            """ 发布控制命令 """
            cmd_msg.acceleration_or_force.x = acc_x
            cmd_msg.acceleration_or_force.y = acc_y
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
    uav_list = ["uav1", "uav2", "uav3", "uav4", "uav5" ]
    
    # 初始化控制器（自动启动定时器）
    controller = FormationTrackController(uav_list)

    rospy.spin()  # 让 ROS 节点保持运行

if __name__ == '__main__':
    main()
