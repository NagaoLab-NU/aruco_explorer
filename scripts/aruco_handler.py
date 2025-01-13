#!/usr/bin/env python
# coding=UTF-8
import rospy
from geometry_msgs.msg import Twist

class ArucoHandler:
    def __init__(self):
        self.marker_pos = None  # 初期化
        self.marker_detected = False

    def aruco_pos_callback(self, msg):
        """
        `/aruco_marker_center`トピックからマーカー位置を取得するコールバック関数
        """
        if msg.x != 0.0 or msg.y != 0.0:
            self.marker_pos = [msg.x, msg.y]
            self.marker_detected = True
            rospy.loginfo("Aruco marker detected at: x={}, y={}".format(msg.x, msg.y))

    def align_with_marker(self, robot_motion, camera_center_x=320, alignment_threshold=50):
        """
        マーカーをカメラの中心に合わせる
        :param robot_motion: RobotMotionクラスのインスタンス
        :param camera_center_x: カメラの画像幅の中央ピクセル値
        :param alignment_threshold: 許容する誤差（ピクセル単位）
        """
        rospy.loginfo("Aligning with Aruco marker...")

        while not rospy.is_shutdown():
            # マーカーが未検出の場合は待機
            if not self.marker_pos:
                rospy.logwarn("No marker detected. Waiting...")
                rospy.sleep(0.1)
                continue

            # 中央とのズレを計算
            offset_x = self.marker_pos[0] - camera_center_x

            # 許容範囲内なら前進して停止
            if abs(offset_x) <= alignment_threshold:
                rospy.loginfo("Aruco marker aligned within threshold.")
                move_cmd = Twist()
                move_cmd.linear.x = 0.3
                robot_motion.cmd_pub.publish(move_cmd)
                rospy.sleep(3)
                robot_motion.stop_motion()
                break

            # 回転速度を設定（ズレに応じて方向を調整）
            move_cmd = Twist()
            move_cmd.angular.z = 0.1 if offset_x < 0 else -0.1
            robot_motion.cmd_pub.publish(move_cmd)

            rospy.sleep(0.1)

        # 最終的に停止
        robot_motion.stop_motion()
        rospy.loginfo("Aruco marker alignment complete.")