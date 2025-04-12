#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move_linear(distance, speed, pub, rate):
    """
    Движение робота вперед/назад на заданное расстояние.
    :param distance: расстояние в метрах (положительное – вперед, отрицательное – назад)
    :param speed: скорость в м/с
    :param pub: паблишер для /cmd_vel
    :param rate: частота публикаций
    """
    move_cmd = Twist()
    move_cmd.linear.x = speed if distance >= 0 else -speed
    move_cmd.angular.z = 0

    t0 = rospy.Time.now()
    duration = abs(distance / speed)
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < duration:
        pub.publish(move_cmd)
        rate.sleep()
    stop_robot(pub)

def rotate(angle, angular_speed, pub, rate):
    """
    Поворот робота на заданный угол.
    :param angle: угол в радианах (положительный – поворот налево, отрицательный – направо)
    :param angular_speed: угловая скорость в рад/с
    :param pub: паблишер для /cmd_vel
    :param rate: частота публикаций
    """
    move_cmd = Twist()
    move_cmd.linear.x = 0
    move_cmd.angular.z = angular_speed if angle >= 0 else -angular_speed

    t0 = rospy.Time.now()
    duration = abs(angle / angular_speed)
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < duration:
        pub.publish(move_cmd)
        rate.sleep()
    stop_robot(pub)

def stop_robot(pub):
    """
    Остановка робота.
    """
    stop_cmd = Twist()
    stop_cmd.linear.x = 0
    stop_cmd.angular.z = 0
    pub.publish(stop_cmd)

def move_robot():
    rospy.init_node('auto_driver')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Гц

    # Задаем параметры движения
    linear_speed = 0.2      # м/с
    angular_speed = 0.5     # рад/с (выберите подходящую угловую скорость)

    # Шаг 1: Движемся 4 метра вперед
    rospy.loginfo("Движение вперед 4 м")
    move_linear(4.0, linear_speed, pub, rate)
    rospy.sleep(1)

    # Шаг 2: Поворот направо на 90° (-π/2 радиан)
    rospy.loginfo("Поворот направо на 90°")
    rotate(-1.6, angular_speed, pub, rate)
    rospy.sleep(1)

    # Шаг 3: Движение вперед 1 метр
    rospy.loginfo("Движение вперед 1 м")
    move_linear(0.5, linear_speed, pub, rate)
    rospy.sleep(1)

    # Шаг 4: Поворот назад на 180° (π радиан)
    rospy.loginfo("Разворот на 180°")
    rotate(3.14, angular_speed, pub, rate)
    rospy.sleep(1)

    # Шаг 5: Движение вперед 3 метра
    rospy.loginfo("Движение вперед 3 м")
    move_linear(2.0, linear_speed, pub, rate)
    rospy.sleep(1)

    rospy.loginfo("Задача выполнена.")

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
