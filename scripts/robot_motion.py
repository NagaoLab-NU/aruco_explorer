#!/usr/bin/env python
# coding=UTF-8
import rospy
import tf
import math
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

class RobotMotion:
    def __init__(self, cmd_pub, linear_speed=0.2, angular_speed=0.5):
        """
        ロボットの移動制御を行うクラス
        """
        self.cmd_pub = cmd_pub
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.tf_listener = tf.TransformListener()  # tfのリスナーを初期化

    def set_speeds(self, linear_speed=None, angular_speed=None):
        if linear_speed is not None:
            self.linear_speed = linear_speed
        if angular_speed is not None:
            self.angular_speed = angular_speed

    def get_current_pose(self):
        """
        現在のロボットの位置と姿勢をtfから取得
        """
        max_retries = 10  # 最大リトライ回数
        for i in range(max_retries):
            try:
                self.tf_listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
                (trans, rot) = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))

                x = trans[0]
                y = trans[1]
                yaw = tf.transformations.euler_from_quaternion(rot)[2]

                return x, y, yaw
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn("Attempt {}/{}: Failed to get transform: {}".format(i + 1, max_retries, e))
                rospy.sleep(1.0)  # 1秒待機
        rospy.logerr("Unable to retrieve transform after {} attempts.".format(max_retries))
        return None, None, None

    def rotate_by_angle(self, angle):
        _, _, current_yaw = self.get_current_pose()
        target_yaw = current_yaw + angle
        # 目標角度を -pi ~ pi の範囲に正規化
        target_yaw = (target_yaw + math.pi) % (2 * math.pi) - math.pi

        rospy.loginfo("Rotating by {:.2f} radians...".format(angle))
        while not rospy.is_shutdown():
            # 現在の角度を取得
            _, _, current_yaw = self.get_current_pose()

            # 角度差を計算し、-pi ~ pi の範囲に正規化
            angle_diff = (target_yaw - current_yaw + math.pi) % (2 * math.pi) - math.pi

            # 回転が完了した場合は停止
            if abs(angle_diff) < 0.05:  # 許容誤差（0.05 rad ≈ 2.87度）
                break

            # 角度差に基づいて回転速度を設定
            move_cmd = Twist()
            move_cmd.angular.z = self.angular_speed * (angle_diff / abs(angle_diff))
            self.cmd_pub.publish(move_cmd)

            # 次のループまで待機
            rospy.sleep(0.1)

        # 回転停止
        self.stop_motion()
        rospy.loginfo("Rotation complete.")

    def move_to_target(self, target_point):
        """
        target_point方向を向く       
        target_pointまで移動
        指定された方向を向く
        指定されたターゲットポイントに移動する
        :param target_point: (x, y, yaw) の形式で与えられる目標地点
        """
        target_x, target_y, target_yaw = target_point["x"], target_point["y"], target_point["yaw"]

        # 現在位置を取得
        current_x, current_y, current_yaw = self.get_current_pose()

        # 目標地点への初期方向を計算
        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)

        # 1. 目標地点の方向を向く
        rospy.loginfo("Rotating to face the target...")
        self.rotate_by_angle(angle_to_target - current_yaw)

        # 2. 目標地点まで直進
        rospy.loginfo("Moving straight to the target...")
        while not rospy.is_shutdown():
            # 現在位置を取得
            current_x, current_y, current_yaw = self.get_current_pose()

            # 距離と方向の計算
            distance_to_target = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
            angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
            
            # 角度差を -π ~ π の範囲に正規化
            angle_diff = (angle_to_target - current_yaw + math.pi) % (2 * math.pi) - math.pi

            # 目標地点に近づいたら停止
            if distance_to_target < 0.1:
                rospy.loginfo("Reached the target position.")
                break

            # 進行方向が目標方向から大きくずれている場合は停止して向きを修正
            if abs(angle_diff) > 0.1:  # 誤差許容範囲（0.1ラジアン ≈ 5.7度）
                rospy.logwarn("Direction off by {:.2f} radians. Adjusting...".format(angle_diff))
                self.stop_motion()
                self.rotate_by_angle(angle_diff)
                continue

            # 直進コマンドを発行
            move_cmd = Twist()
            move_cmd.linear.x = self.linear_speed
            self.cmd_pub.publish(move_cmd)
            rospy.sleep(0.1)

        # 停止
        self.stop_motion()

        # 3. 目標方向を向く
        rospy.loginfo("Adjusting to final orientation...")
        self.rotate_by_angle(target_yaw - current_yaw)

        rospy.loginfo("Successfully moved to the target point: x={}, y={}, yaw={}".format(
            target_x, target_y, target_yaw))

    def stop_motion(self):
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
