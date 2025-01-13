#!/usr/bin/env python
# coding=UTF-8
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from collections import deque 

class ObstacleAvoidance:
    def __init__(self, cmd_pub, laser_topic="/scan", distance_threshold=0.2, point_threshold=20, history_size=5):
        """
        障害物回避を行うクラス
        """
        self.cmd_pub = cmd_pub
        self.laser_topic = laser_topic
        self.distance_threshold = distance_threshold  # 20cm以内を検知
        self.point_threshold = point_threshold  # 閾値以上の点数で回避動作
        self.history_size = history_size  # 平均を取るための履歴のサイズ
        self.obstacle_detected = False

        # 点群データの履歴を保持するキュー
        self.scan_history = deque(maxlen=self.history_size)

        rospy.Subscriber(self.laser_topic, LaserScan, self.laser_callback)

    def laser_callback(self, msg):
        """
        レーザースキャンデータを取得するコールバック関数
        """
        self.scan_history.append(np.array(msg.ranges))  # キューに点群データを追加

    def get_average_scan(self):
        """
        点群データの平均を計算
        """
        if len(self.scan_history) == 0:
            rospy.logwarn("No laser scan data in history.")
            return None

        # キュー内のデータの平均を計算
        return np.mean(np.array(self.scan_history), axis=0)

    def detect_obstacles(self):
        """
        障害物を検知し、フラグを設定する
        """
        average_scan = self.get_average_scan()
        if average_scan is None:
            return None

        # セクターごとの点数をカウント
        sector_size = len(average_scan) // 18  # 180度を10度ごとに分割
        obstacle_counts = [
            np.sum(average_scan[i * sector_size:(i + 1) * sector_size] < self.distance_threshold)
            for i in range(18)
        ]

        rospy.loginfo("Obstacle counts per sector: {}".format(obstacle_counts))

        # 最大値を持つセクターをすべて取得
        max_count = max(obstacle_counts)
        candidates = [i for i, count in enumerate(obstacle_counts) if count == max_count]

        # 中央（セクターID 9）に最も近いセクターを選ぶ
        selected_sector = min(candidates, key=lambda x: abs(x - 9))

        rospy.loginfo("Selected sector for avoidance: {}".format(selected_sector))

        if max_count > self.point_threshold:
            self.obstacle_detected = True
            return selected_sector  # 最大の障害物のセクターIDを返す
        else:
            self.obstacle_detected = False
            return None

    def avoid_obstacle(self, sector_id):
        """
        障害物を回避する
        :param sector_id: 障害物が多いセクターのID (0-17)
        """
        rospy.loginfo("Avoiding obstacle detected in sector {}".format(sector_id))

        # 反対方向に回避する回転量を計算
        if sector_id < 9:
            rotate_direction = 1  # 左回避
            rospy.loginfo("[Avoid]: turn left")
        else:
            rotate_direction = -1  # 右回避
            rospy.loginfo("[Avoid]: turn right")

        # 回転コマンドを送信
        move_cmd = Twist()
        move_cmd.angular.z = rotate_direction * 0.5  # 回転速度
        self.cmd_pub.publish(move_cmd)
        rospy.sleep(1.0)  # 回転時間（必要に応じて調整）
        move_cmd = Twist()
        move_cmd.linear.x = 0.2  # 回転速度
        self.cmd_pub.publish(move_cmd)
        rospy.sleep(1.0) 

        # 停止
        self.stop_motion()

    def stop_motion(self):
        """
        動作を停止する
        """
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)

    def run(self):
        """
        障害物回避のメインループ
        """
        rospy.loginfo("Starting obstacle avoidance...")
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown():
            sector_id = self.detect_obstacles()
            if self.obstacle_detected and sector_id is not None:
                self.avoid_obstacle(sector_id)
            rate.sleep()
