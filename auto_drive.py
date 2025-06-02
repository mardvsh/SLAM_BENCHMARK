#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move_linear(distance, speed, pub, rate):
    move_cmd = Twist()
    move_cmd.linear.x = speed
    move_cmd.angular.z = 0

    t0 = rospy.Time.now()
    duration = distance / speed
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < duration:
        pub.publish(move_cmd)
        rate.sleep()
    stop_robot(pub)

def rotate(angle_deg, angular_speed_deg, pub, rate, clockwise=False):
    """
    Поворот робота на заданный угол.
    :param angle_deg: угол в градусах
    :param angular_speed_deg: угловая скорость в град/с
    :param pub: паблишер /cmd_vel
    :param rate: частота публикации
    :param clockwise: True - по часовой стрелке, False - против
    """
    move_cmd = Twist()
    angular_speed_rad = angular_speed_deg * 3.1416 / 180
    move_cmd.angular.z = -angular_speed_rad if clockwise else angular_speed_rad

    t0 = rospy.Time.now()
    duration = angle_deg / angular_speed_deg
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < duration:
        pub.publish(move_cmd)
        rate.sleep()
    stop_robot(pub)

def stop_robot(pub):
    stop_cmd = Twist()
    stop_cmd.linear.x = 0
    stop_cmd.angular.z = 0
    pub.publish(stop_cmd)

def move_robot():
    rospy.init_node('simple_forward_driver')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Гц

    # Параметры
    linear_speed = 0.2  # м/с
    distance = 4.0      # м

    rospy.loginfo("Движение вперед на 4 метра")
    move_linear(distance, linear_speed, pub, rate)

    rospy.loginfo("Поворот налево на 90 градусов")
    rotate(angle_deg=90, angular_speed_deg=90, pub=pub, rate=rate)

    rospy.loginfo("Проезд вперед на 0.5 метра")
    move_linear(0.5, linear_speed, pub, rate)

    rospy.loginfo("Остановка")

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
