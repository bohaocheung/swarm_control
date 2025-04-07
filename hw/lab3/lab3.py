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
        self.ugv_list = ["ugv1", "ugv2"]
        
        self.data = {
            "uav1": {"pose": None, "velocity": None, "hpx": 0.0, "hpy": 0.0, "hvx": 0.0, "hvy": 0.0, "ka":0.0, "kn": 3, "ki": 0.5},
            "uav2": {"pose": None, "velocity": None, "hpx": 0.0, "hpy": 0.0, "hvx": 0.0, "hvy": 0.0, "ka":0.0, "kn": 3, "ki": 1},
            "uav3": {"pose": None, "velocity": None, "hpx": 0.0, "hpy": 0.0, "hvx": 0.0, "hvy": 0.0, "ka":0.0, "kn": 3, "ki": 2},
            "uav4": {"pose": None, "velocity": None, "hpx": 0.0, "hpy": 0.0, "hvx": 0.0, "hvy": 0.0, "ka":0.0, "kn": 5, "ki": 0},
            "uav5": {"pose": None, "velocity": None, "hpx": 0.0, "hpy": 0.0, "hvx": 0.0, "hvy": 0.0, "ka":0.0, "kn": 5, "ki": 1},
            "uav6": {"pose": None, "velocity": None, "hpx": 0.0, "hpy": 0.0, "hvx": 0.0, "hvy": 0.0, "ka":0.0, "kn": 5, "ki": 2.5},
            "uav7": {"pose": None, "velocity": None, "hpx": 0.0, "hpy": 0.0, "hvx": 0.0, "hvy": 0.0, "ka":0.0, "kn": 5, "ki": 3},
            "uav8": {"pose": None, "velocity": None, "hpx": 0.0, "hpy": 0.0, "hvx": 0.0, "hvy": 0.0, "ka":0.0, "kn": 5, "ki": 3.5}
        }
        self.ugv_data = {
            "ugv1": {"pose1": None, "pose2": None, "velocity": None},
            "ugv2": {"pose1": None, "pose2": None, "velocity": None}
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
        
        for i, ugv in enumerate(self.ugv_list):
            # 订阅位姿和速度话题
            rospy.Subscriber(f'/{ugv}/pose', PoseStamped,
            lambda msg, idx=i: self.ugv_pose_callback(msg, idx))    
        
        # 定时器，每 1/20 秒（50ms）调用一次 publish_control_commands
        self.timer = rospy.Timer(rospy.Duration(1.0 / 20), self.publish_control_commands)

        # 记录启动时间
        self.start_time = rospy.Time.now()

        self.obstacles = [
            {'x': -6.0, 'y': 22.0, 'radius': 1.5},
            {'x': -3.0, 'y': 16.0, 'radius': 1.5},
            {'x':  4.0, 'y': 22.0, 'radius': 1.5},
            {'x': 9.0, 'y': 16.0, 'radius': 1.5}
        ]

    def ugv_pose_callback(self, msg, idx):
        now = rospy.Time.now()
        ugv_name = self.ugv_list[idx]

        # 更新位置信息
        self.ugv_data[ugv_name]["pose1"] = self.ugv_data[ugv_name]["pose2"]
        self.ugv_data[ugv_name]["pose2"] = msg
        p1 = self.ugv_data[ugv_name]["pose1"]
        p2 = self.ugv_data[ugv_name]["pose2"]

        # 计算速度
        if p1 == None or p2 == None or self.ugv_last_time == None:
            self.ugv_last_time = now
            return 
        p1 = p1.pose.position
        p2 = p2.pose.position
        self.ugv_data[ugv_name]["velocity"] = {
            "vx":(p2.x - p1.x)/(now - self.ugv_last_time).to_sec(),
            "vy":(p2.y - p1.y)/(now - self.ugv_last_time).to_sec()
        }
        self.ugv_last_time = now

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
    
        """ 获取领导者信息 """
        if self.ugv_data["ugv1"]["pose"] == None or self.ugv_data["ugv1"]["velocity"] == None or self.ugv_data["ugv2"]["pose"] == None or self.ugv_data["ugv2"]["velocity"] == None:
            return

        # 包围圈半径缩小函数
        rd = 4
        r = max(1, 30 - rd * elapsed_time)
        w = 0.05
        # 潜在碰撞距离阈值
        d_thresh_u = 1
        d_thresh_s = 3
        # 避障控制权重
        kc_min_u = 4
        kc_min_s = 45
        # 阶段切换距离阈值
        switch = 3
        # 编队跟踪
        g1_ka = 80 / 180
        g2_ka = 80 / 180
        # 控制参数
        c = 4.0
        b = 2.0
        sigma = 0.01

        # 分组
        group1 = ["uav1", "uav2", "uav3"]
        group2 = ["uav4", "uav5", "uav6", "uav7", "uav8"]

        for i,uav in enumerate(self.uav_list):
            """ 发布控制指令（仅使用 x/y 加速度和 z 位置，偏航角固定为 0） """
            tag = False

            """ 确定跟踪扇形角度 """
            if uav in group1:
                self.data[uav]["ka"] = g1_ka
            else:
                self.data[uav]["ka"] = g2_ka

            if self.data[uav]["pose"] is None or self.data[uav]["velocity"] is None:
                continue
            
            """ 确定 uav 信息"""
            pos_uav = self.data[uav]["pose"].pose.position
            vel_uav = self.data[uav]["velocity"].twist.linear

            """ 确定目标 """
            ugv_str = "ugv1" if i <= 2 else "ugv2"
            pos_ugv = self.ugv_data[ugv_str]["pose"].pose.position
            vel_ugv = self.ugv_data[ugv_str]["velocity"].twist.linear

            """ 确定组别 """
            group = group1 if i <= 2 else group2

            """ 计算编队状态下的偏移向量 """
            kr = r
            ka = self.data[uav]["ka"]
            ki = self.data[uav]["ki"]
            kn = self.data[uav]["kn"]

            """ 是否切换为打击阶段 """
            d2c = math.sqrt((pos_uav.x - pos_ugv.x) ** 2 + (pos_uav.y - pos_ugv.y) ** 2)
            if d2c <= switch:
                kr = 1
                ka = 2
                tag = True

            self.data[uav]["hpx"] = kr * math.cos(w * elapsed_time + ka * math.pi * ki / kn)
            self.data[uav]["hpy"] = kr * math.sin(w * elapsed_time + ka * math.pi * ki / kn)
            self.data[uav]["hvx"] = -w * kr * math.sin(w * elapsed_time + ka * math.pi * ki / kn)
            self.data[uav]["hvy"] = w * kr * math.cos(w * elapsed_time + ka * math.pi * ki / kn)


            """ 计算自身误差 """
            delta_x = - 0.3416 * (pos_uav.x - self.data[uav]["hpx"] - pos_ugv.x) - 0.7330 * (vel_uav.x - self.data[uav]["hvx"] - vel_ugv.x)
            delta_y = - 0.3416 * (pos_uav.y - self.data[uav]["hpy"] - pos_ugv.y) - 0.7330 * (vel_uav.y - self.data[uav]["hvy"] - vel_ugv.y)

            """ 计算组内其他节点信息 """
            for other_uav in group:
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
                
                if tag:
                    continue
                
                """ 无人机组内碰撞检测 """
                dx = pos_uav.x - pos_other.x
                dy = pos_uav.y - pos_other.y
                d = math.sqrt(dx ** 2 + dy ** 2) 
                if  d <= d_thresh_u:
                    d = d_thresh_u - d
                    theta = math.atan2(abs(dy), abs(dx))
                    abs_x = d * math.cos(theta) 
                    abs_y = d * math.sin(theta)
                    kc_u = kc_min_u / d  
                    delta_x += kc_u * abs_x if dx > 0 else -abs_x * kc_u
                    delta_y += kc_u * abs_y if dy > 0 else -abs_y * kc_u

        
            """ 无人机与圆柱体碰撞检测 """
            for o in self.obstacles:
                dx = pos_uav.x - o['x']
                dy = pos_uav.y - o['y']
                d = math.sqrt(dx ** 2 + dy ** 2) 
                d_thresh_o = d_thresh_s + o['radius']
                if  d <= d_thresh_o:
                    d = d_thresh_o - d
                    theta = math.atan2(abs(dy), abs(dx))
                    abs_x = d * math.cos(theta) 
                    abs_y = d * math.sin(theta)
                    kc_s = kc_min_s / d 
                    delta_x += kc_s * abs_x if dx > 0 else -abs_x * kc_s
                    delta_y += kc_s * abs_y if dy > 0 else -abs_y * kc_s


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
            
            phi = c + b / (math.sqrt(delta_x**2 + delta_y**2) + sigma)

            acc_x = phi * delta_x
            acc_y = phi * delta_y

            """ 计算时变编队补偿控制输入 """
            acc_x += w**2 * kr * math.cos(w * elapsed_time + ka * math.pi * ki / kn)
            acc_y += w**2 * kr * math.sin(w * elapsed_time + ka * math.pi * ki / kn)

            """ 发布控制命令 """
            cmd_msg.acceleration_or_force.x = acc_x
            cmd_msg.acceleration_or_force.y = acc_y
            self.cmd_publishers[uav].publish(cmd_msg)

            """ 发布控制误差 """
            error_msg = TwistStamped()
            error_msg.twist.linear.x = pos_uav.x - self.data[uav]["hpx"] - pos_ugv.x # x轴位置误差
            error_msg.twist.linear.y = pos_uav.y - self.data[uav]["hpy"] - pos_ugv.y # y轴位置误差
            error_msg.twist.angular.x = vel_uav.x - self.data[uav]["hvx"]- vel_ugv.x # x轴速度误差
            error_msg.twist.angular.y = vel_uav.y - self.data[uav]["hvy"]- vel_ugv.y # y轴速度误差

            self.error_publishers[uav].publish(error_msg)

def main():
    rospy.init_node('formation_controller_node', anonymous=True)

    # 设定要监听和控制的无人机
    uav_list = ["uav1", "uav2", "uav3", "uav4", "uav5", "uav6", "uav7", "uav8"]
    
    # 初始化控制器（自动启动定时器）
    controller = FormationTrackController(uav_list)

    rospy.spin()  # 让 ROS 节点保持运行

if __name__ == '__main__':
    main()
