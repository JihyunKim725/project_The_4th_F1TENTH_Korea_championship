#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import threading
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

class JoystickVehicleControl:
    def __init__(self):
        rospy.init_node("joystick_vehicle_control", anonymous=True)

        # ROS Subscribers & Publishers
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.drive_pub = rospy.Publisher("/ackermann_cmd", AckermannDriveStamped, queue_size=1)

        # Parameters
        self.max_speed = 1  # 최대 속도 (1m/s)
        self.max_steering_angle = 1  # 최대 조향 각도
        self.speed = 0  # 현재 속도
        self.steering_angle = 0.0  # 현재 조향 각도

    def joy_callback(self, joy_msg):
        """조이스틱 입력 처리"""
        # 축(Axis) 조작 → 기본 속도 및 조향 설정
        self.speed = joy_msg.axes[1] * self.max_speed  
        self.steering_angle = joy_msg.axes[3] / 2 * self.max_steering_angle  

        # 버튼(0번) 클릭 시 1m/s로 10초간 유지
        if joy_msg.buttons[0] == 1:
            rospy.loginfo("Button 0 pressed! Moving at 1m/s for 10 seconds.")
            self.speed = 1.0  # 1m/s로 설정
            self.publish_drive()  # 즉시 속도 반영
            threading.Timer(10.0, self.stop_vehicle).start()  # 10초 후 정지

    def publish_drive(self):
        """Ackermann 메시지 발행"""
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.speed
        drive_msg.drive.steering_angle = self.steering_angle
        self.drive_pub.publish(drive_msg)

    def stop_vehicle(self):
        """차량 정지 (10초 후 실행)"""
        rospy.loginfo("Stopping vehicle after 10 seconds.")
        self.speed = 0
        self.publish_drive()

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    joystick_control = JoystickVehicleControl()
    joystick_control.run()
