#!/usr/bin/env python
# coding=UTF-8
import rospy
import rospkg
import math
from robot_motion import RobotMotion
from aruco_handler import ArucoHandler
from utils import load_config
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from obstacle_avoidance import ObstacleAvoidance 

class AreaExplorer:
    def __init__(self):
        rospy.init_node('area_explorer', anonymous=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospack = rospkg.RosPack()
        config_path = rospack.get_path('area_explorer') + '/config/config.yaml'
        self.config = load_config(config_path)
        rospy.loginfo("Configuration loaded.")

        self.robot_motion = RobotMotion(self.cmd_pub)
        self.aruco_handler = ArucoHandler()
        # 障害物回避のインスタンスを作成
        self.obstacle_avoidance = ObstacleAvoidance(self.cmd_pub)

        self.start_position = self.config['start_position']
        self.areas = self.config['areas']

        rospy.Subscriber('/aruco_marker_center', Point, self.aruco_handler.aruco_pos_callback)
        rospy.Subscriber('/step_detected', Bool, self.step_callback)
        rospy.Subscriber('/start', Bool, self.start_callback)  # /startトピックの購読
        self.step_detected = False
        self.start_flag = False  # デフォルトでは動かない

    def step_callback(self, msg):
        self.step_detected = msg.data
        if self.step_detected:
            rospy.loginfo("Step detected!")

    def start_callback(self, msg):
        self.start_flag = msg.data
        if self.start_flag:
            rospy.loginfo("Start signal received. Beginning exploration...")

    def wait_for_start(self):
        """
        /startトピックからTrueが来るまで待機する
        """
        rospy.loginfo("Waiting for start signal...")
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            if self.start_flag:
                return  # start_flagがTrueになったら抜ける
            rate.sleep()

    def move_to_end_with_step_detection(self, target_point):
        """
        エリアの終了地点に向かって移動し、途中で step_detected が True なら停止する
        :param target_point: (x, y, yaw) の形式で与えられる目標地点
        """
        target_x, target_y, target_yaw = target_point["x"], target_point["y"], target_point["yaw"]

        # 現在位置を取得
        current_x, current_y, current_yaw = self.robot_motion.get_current_pose()

        # 目標地点への初期方向を計算
        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)

        # 1. 目標地点の方向を向く
        rospy.loginfo("Rotating to face the target...")
        self.robot_motion.rotate_by_angle(angle_to_target - current_yaw)

        # 2. 目標地点まで直進（step_detected を監視）
        rospy.loginfo("Moving straight to the target with step detection...")
        while not rospy.is_shutdown():
            # 現在位置を取得
            current_x, current_y, current_yaw = self.robot_motion.get_current_pose()

            # 距離と方向の計算
            distance_to_target = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
            angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
            
            # 角度差を -π ~ π の範囲に正規化
            angle_diff = (angle_to_target - current_yaw + math.pi) % (2 * math.pi) - math.pi

            # 目標地点に近づいたら停止
            if distance_to_target < 0.1:
                rospy.loginfo("Reached the target position.")
                break

            # **障害物の確認**
            sector_id = self.obstacle_avoidance.detect_obstacles()
            if self.obstacle_avoidance.obstacle_detected and sector_id is not None:
                rospy.logwarn("Obstacle detected in sector {}. Avoiding...".format(sector_id))
                self.obstacle_avoidance.avoid_obstacle(sector_id)
                rospy.loginfo("Obstacle avoided. Resuming movement.")

            # 途中で step_detected が True になった場合に停止
            if self.step_detected:
                rospy.logwarn("Step detected! Stopping movement.")
                self.robot_motion.stop_motion()
                self.step_detected = False
                return  True

            # 進行方向が目標方向から大きくずれている場合は向きを修正
            if abs(angle_diff) > 0.1:  # 誤差許容範囲（0.1ラジアン ≈ 5.7度）
                rospy.logwarn("Direction off by {:.2f} radians. Adjusting...".format(angle_diff))
                self.robot_motion.stop_motion()
                self.robot_motion.rotate_by_angle(angle_diff)
                continue

            # 直進コマンドを発行
            move_cmd = Twist()
            move_cmd.linear.x = self.robot_motion.linear_speed
            self.robot_motion.cmd_pub.publish(move_cmd)
            rospy.sleep(0.1)

        # 停止
        self.robot_motion.stop_motion()

        # 3. 目標方向を向く
        rospy.loginfo("Adjusting to final orientation...")
        self.robot_motion.rotate_by_angle(target_yaw - angle_to_target)

        rospy.loginfo("Successfully moved to the target point with step detection.")

        return False
    
    def search_for_aruco_marker(self):
        """
        Arucoマーカーを探す動作
        - 0.3m後退
        - 左右を探索
        """
        rospy.loginfo("Searching for Aruco marker...")

        # **1. 0.3m後退**
        rospy.loginfo("Backing up 0.3m...")
        move_cmd = Twist()
        move_cmd.linear.x = -0.2  # 後退速度
        distance_moved = 0.0

        while distance_moved < 0.2 and not rospy.is_shutdown():
            self.cmd_pub.publish(move_cmd)
            rospy.sleep(0.1)
            distance_moved += 0.2 * 0.1  # 速度(0.2 m/s) × 時間(0.1秒)

        # 停止
        self.robot_motion.stop_motion()
        rospy.sleep(1)

        # **2. 左右を探索**
        rospy.loginfo("Rotating left 30 degrees...")
        self.robot_motion.rotate_by_angle(math.radians(30))  # 左に30度回転
    
        # 最大回転角（60度 = π/3 ラジアン）
        max_rotation = math.radians(60)
        rotated_angle = 0.0  # 現在の回転角度

        move_cmd = Twist()
        move_cmd.angular.z = -0.2  # 右回転速度

        while rotated_angle < max_rotation and not rospy.is_shutdown():
            # マーカーが検出された場合
            if self.aruco_handler.marker_detected:
                rospy.loginfo("Aruco marker detected! Aligning with marker...")
                self.aruco_handler.align_with_marker(self.robot_motion)
                return  # 中央調整後に関数を終了

            # 右回転のコマンドを発行
            self.cmd_pub.publish(move_cmd)
            rospy.sleep(0.1)
            rotated_angle += abs(move_cmd.angular.z) * 0.1  # 角速度 * 時間で回転角を更新

        # 最大回転範囲を超えた場合
        rospy.logwarn("Aruco marker not detected within 60 degrees.")
        self.robot_motion.stop_motion()

    def explore_area(self):
        # エリアごとに順番に探索を行う
        for area_id, points in self.areas.items():
            rospy.loginfo("Exploring Area {}...".format(area_id))
            
            # エリアのスタート地点に移動
            rospy.loginfo("Moving to the start point of Area {}...".format(area_id))
            self.robot_motion.move_to_target(points["start"])

            # エリアの終了地点に向けて移動（途中で step_detected が True なら停止）
            rospy.loginfo("Moving to the end point of Area {} with step detection...".format(area_id))
            aruco_detect_flag = self.move_to_end_with_step_detection(points["end"])
            if aruco_detect_flag:
                rospy.loginfo("Step detected, starting Aruco marker search...")
                self.search_for_aruco_marker()
            
            rospy.loginfo("Moving to the start point")
            self.robot_motion.move_to_target(
                {"x": self.config["start_position"]["x"],
                 "y": self.config["start_position"]["y"],
                 "yaw": self.config["start_position"]["yaw"]},
            )

            # アルコマーカーを置くために少しバックしてから回転
            move_cmd = Twist()
            move_cmd.linear.x = -0.3
            self.cmd_pub.publish(move_cmd)
            rospy.sleep(1)
            self.robot_motion.rotate_by_angle(math.radians(180)) 
            self.aruco_handler.marker_detected = False
            rospy.loginfo("Reset step detected flag") 

            rospy.loginfo("Area {} exploration complete.".format(area_id))

    def run(self):
        rospy.loginfo("Starting area exploration...")
        self.wait_for_start()  # 開始信号を待つ
        self.explore_area()
        rospy.loginfo("Exploration complete.")

if __name__ == '__main__':
    try:
        explorer = AreaExplorer()
        explorer.run()
    except rospy.ROSInterruptException:
        pass