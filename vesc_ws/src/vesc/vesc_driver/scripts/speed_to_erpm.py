
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

# 모터 및 차량 기본 설정
WHEEL_DIAMETER = 0.074  # 바퀴 지름 (단위: m)
GEAR_RATIO = 3.857143  # 기어비
POLE_PAIRS = 2  # BLDC 모터 Pole Pair 수

# 속도를 ERPM으로 변환
def speed_to_erpm(speed_m_s):
    wheel_circumference = WHEEL_DIAMETER * 3.141592
    motor_rpm = (speed_m_s * 60) / wheel_circumference
    erpm = motor_rpm * POLE_PAIRS * GEAR_RATIO
    return erpm

# ERPM을 출력하는 콜백 함수
def vesc_callback(data):
    current_erpm = data.state.speed
    rospy.loginfo(f"Current ERPM: {current_erpm}")

# 속도 명령을 수신하는 콜백 함수
def cmd_vel_callback(data):
    target_speed = data.linear.x  # m/s
    target_erpm = speed_to_erpm(target_speed)
    rospy.loginfo(f"Target Speed: {target_speed:.2f} m/s, Target ERPM: {target_erpm:.2f}")

# ROS 노드 실행
def main():
    rospy.init_node("vesc_erpm_publisher", anonymous=True)

    # VESC에서 현재 ERPM 값 구독
    rospy.Subscriber("/vesc/odom", AckermannDriveStamped, vesc_callback)

    # 차량 속도 명령 구독
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)

    rospy.spin()

if __name__ == "__main__":
    main()
