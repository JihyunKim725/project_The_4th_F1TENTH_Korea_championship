import math
import rospy
import numpy as np

# ì‚¬ìš©ìž ìž…ë ¥ê°’ ì„¤ì •
wheel_diameter = 74  # mm (íœ  ì§ê²½)
gear_ratio = 3.857143  # ê¸°ì–´ë¹„ (ìŠ¤í¼ ê¸°ì–´ / í”¼ë‹ˆì–¸ ê¸°ì–´)
motor_pole_pairs = 2  # JLB3670-2500KV ëª¨í„° (4 Pole = 2 Pairs)
from ackermann_msgs.msg import AckermannDriveStamped
class FollowTheGap :
    def __init__(self) :
        rospy.init_node("odom_parameter", anonymous=True)
        self.drive_pub = rospy.Publisher("/ackermann_cmd", AckermannDriveStamped, queue_size=1)
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed_ms


def speed_to_erpm(speed_kmh):
    wheel_circumference = wheel_diameter * math.pi  # mm ë‹¨ìœ„
    wheel_rpm = (speed_kmh * 1000) / wheel_circumference  # ë°”í€´ RPM
    motor_rpm = wheel_rpm * gear_ratio  # ê¸°ì–´ë¹„ ì ìš©
    erpm = motor_rpm * motor_pole_pairs  # ERPM ë³€í™˜
    return erpm

# íŠ¹ì • ì†ë„ì—ì„œ ERPM ì¶œë ¥
speed_ms = float(input("ì†ë„(m/s)ë¥¼ ìž…ë ¥í•˜ì„¸ìš”:"))
speed_kmh = speed_ms * 3.6
speed_to_erpm_gain = speed_to_erpm(speed_kmh)
print(f"{speed_ms} m/sì—ì„œ speed_to_erpm_gain ê°’: {speed_to_erpm_gain}")
3