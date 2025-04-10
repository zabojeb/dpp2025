#!/usr/bin/env python3
"""
Этот скрипт интегрирует функциональность ROS (Robot Operating System) с веб-сокетами через Socket.IO,
обеспечивая следующие возможности:
  - Получение и отправка команд движения (cmd_vel) через ROS.
  - Подписка на данные одометрии ROS и их преобразование в словарь для передачи клиентам.
  - Управление сервоприводами: изменение значений, отправка через CAN-шину.
  - Обработка событий подключения/отключения клиентов и обмен данными через Socket.IO.
"""

import rospy
# Для формирования и публикации сообщений движения.
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry        # Для приёма данных одометрии робота.
import socketio                        # Для организации Socket.IO сервера.
# Асинхронная библиотека для поддержки Socket.IO.
import eventlet
# Для развёртывания WSGI-приложения с Socket.IO.
import eventlet.wsgi
# Для упаковки значений (например, сервоприводов) в бинарный формат.
import struct
import can                             # Для отправки сообщений по CAN-шине.

# Глобальные переменные для взаимодействия с ROS и управления сервоприводами:
pub = None             # Паблишер ROS для отправки команд движения (cmd_vel).
# Переменная для хранения последнего полученного сообщения одометрии.
latest_odom = None
# Список из 7 значений: первые 6 – углы сервоприводов (0..180 градусов), 7-й – скорость для сервопривода с непрерывным вращением.
servo_values = [90.0] * 7

# Функция-обработчик сообщений одометрии от ROS.


def odom_callback(msg):
    """
    Вызывается при получении нового сообщения одометрии.
    Сохраняет полученное сообщение в глобальной переменной latest_odom для последующей передачи клиентам.
    """
    global latest_odom
    latest_odom = msg

# Функция для преобразования сообщения Odometry в словарь.


def odom_to_dict(msg):
    """
    Преобразует ROS-сообщение Odometry в формат словаря для удобства последующей передачи по Socket.IO.

    Параметры:
      msg: объект сообщения Odometry

    Возвращает:
      Словарь с данными одометрии, включающий заголовок, позу и скорость (twist) с соответствующими ковариациями.
    """
    return {
        "header": {
            "seq": msg.header.seq,
            "stamp": {
                "secs": msg.header.stamp.secs,
                "nsecs": msg.header.stamp.nsecs
            },
            "frame_id": msg.header.frame_id
        },
        "child_frame_id": msg.child_frame_id,
        "pose": {
            "pose": {
                "position": {
                    "x": msg.pose.pose.position.x,
                    "y": msg.pose.pose.position.y,
                    "z": msg.pose.pose.position.z
                },
                "orientation": {
                    "x": msg.pose.pose.orientation.x,
                    "y": msg.pose.pose.orientation.y,
                    "z": msg.pose.pose.orientation.z,
                    "w": msg.pose.pose.orientation.w
                }
            },
            "covariance": list(msg.pose.covariance)
        },
        "twist": {
            "twist": {
                "linear": {
                    "x": msg.twist.twist.linear.x,
                    "y": msg.twist.twist.linear.y,
                    "z": msg.twist.twist.linear.z
                },
                "angular": {
                    "x": msg.twist.twist.angular.x,
                    "y": msg.twist.twist.angular.y,
                    "z": msg.twist.twist.angular.z
                }
            },
            "covariance": list(msg.twist.covariance)
        }
    }

# Функция для публикации команды cmd_vel в ROS.


def publish_cmd_vel(data):
    """
    Принимает словарь с данными о движении, формирует из них объект Twist,
    публикует его в топик /cmd_vel и возвращает результат выполнения.

    Параметры:
      data: словарь с полями 'linear' и 'angular', содержащими координаты скорости.

    Возвращает:
      (True, сообщение) в случае успешной публикации или (False, сообщение об ошибке)
    """
    twist = Twist()
    try:
        # Преобразуем строки в float и присваиваем линейные и угловые скорости.
        twist.linear.x = float(data["linear"]["x"])
        twist.linear.y = float(data["linear"]["y"])
        twist.linear.z = float(data["linear"]["z"])
        twist.angular.x = float(data["angular"]["x"])
        twist.angular.y = float(data["angular"]["y"])
        twist.angular.z = float(data["angular"]["z"])
    except KeyError as e:
        rospy.logerr("Отсутствует параметр: {}".format(e))
        return False, f"Отсутствует параметр: {e}"
    except Exception as e:
        rospy.logerr("Ошибка обработки данных: {}".format(e))
        return False, f"Ошибка обработки данных: {e}"

    # Публикуем сформированное сообщение.
    pub.publish(twist)
    rospy.loginfo("Опубликовано: {}".format(data))
    return True, "Команда опубликована"


def send_servo_values():
    """
    Упаковывает глобальный список servo_values в бинарный формат с помощью struct,
    формирует CAN-сообщение и отправляет его по шине.
    Используется идентификатор 0x700, стандартный 11-битный ID и включён режим CAN-FD.
    """
    # Упаковка 7 значений float в бинарный формат ('7f' обозначает 7 чисел с плавающей точкой).
    data = struct.pack('7f', *servo_values)
    msg = can.Message(
        arbitration_id=0x700,    # Идентификатор сообщения для CAN.
        data=data,
        # Использование стандартного 11-битного идентификатора.
        is_extended_id=False,
        is_fd=True               # Включение CAN-FD.
    )
    try:
        # Создаем объект шины для канала 'can0' с типом 'socketcan' и поддержкой CAN-FD.
        with can.interface.Bus(channel='can0', bustype='socketcan', fd=True) as bus:
            bus.send(msg)
            rospy.loginfo("Отправлено: {}".format(servo_values))
    except can.CanError as e:
        rospy.logerr("Ошибка отправки CAN-сообщения: {}".format(e))


def update_servo(servo_id, value):
    """
    Обновляет значение сервопривода по заданному индексу и отправляет обновленные данные через CAN-шину.

    Параметры:
      servo_id: индекс сервопривода (от 0 до 6)
      value: новое значение (угол или скорость), приводится к float

    Возвращает:
      True, если значение обновлено, иначе False.
    """
    if servo_id < 0 or servo_id >= len(servo_values):
        return False
    servo_values[servo_id] = float(value)

    # После обновления значений, отправляем их через CAN.
    send_servo_values()
    return True


# Создание Socket.IO сервера с поддержкой CORS.
sio = socketio.Server(cors_allowed_origins='*')
# Оборачивание Socket.IO сервера в WSGI-приложение для поддержки HTTP.
app = socketio.WSGIApp(sio)

# ----------------- Обработчики событий Socket.IO -----------------


@sio.event
def connect(sid, environ):
    """
    Обработчик события подключения нового клиента.
    При подключении:
      - Регистрируем подключение в логах ROS.
      - Отправляем клиенту текущие значения сервоприводов.
    """
    rospy.loginfo("Клиент подключился: {}".format(sid))

    # Отправка события "servo_update" конкретному клиенту.
    sio.emit("servo_update", {"servo_values": servo_values}, room=sid)


@sio.event
def disconnect(sid):
    """
    Обработчик события отключения клиента.
    Регистрирует в логах факт отключения.
    """
    rospy.loginfo("Клиент отключился: {}".format(sid))


@sio.event
def set_servo(sid, data):
    """
    Обработчик события изменения значения сервопривода.
    Ожидает словарь с параметрами:
      - servo_id: индекс сервопривода.
      - value: новое значение.
    При отсутствии необходимых полей отправляется сообщение об ошибке.

    После успешного обновления, рассылает обновлённые значения всем подключенным клиентам.
    """
    servo_id = data.get("servo_id")
    value = data.get("value")
    if servo_id is None or value is None:
        sio.emit("error", {"error": "Missing servo_id or value"}, room=sid)
        return
    if not update_servo(servo_id, value):
        sio.emit("error", {"error": "Invalid servo_id"}, room=sid)
        return

    # Рассылка обновлённых значений сервоприводов всем клиентам.
    sio.emit("servo_update", {"servo_values": servo_values})


@sio.event
def cmd_vel(sid, data):
    """
    Обработчик события получения команды движения (cmd_vel) от клиента.
    Формирует сообщение типа Twist, публикует его в топик /cmd_vel ROS и отправляет ответ клиенту с результатом операции.

    Параметры:
      data: словарь с полями 'linear' и 'angular' для задания скорости движения.
    """
    success, message = publish_cmd_vel(data)
    if success:
        sio.emit("cmd_vel_response", {
                 "status": "success", "message": message}, room=sid)
    else:
        sio.emit("cmd_vel_response", {
                 "status": "error", "message": message}, room=sid)


@sio.event
def get_odom(sid):
    """
    Обработчик события запроса текущих данных одометрии.
    Если данные доступны, преобразует их в словарь и отправляет клиенту,
    иначе отправляет сообщение об ошибке.
    """
    if latest_odom is None:
        sio.emit("odom_response", {
                 "status": "error", "message": "Данные одометрии недоступны"}, room=sid)
    else:
        odom_data = odom_to_dict(latest_odom)
        sio.emit("odom_response", odom_data, room=sid)


# ----------------- Инициализация ROS и запуск Socket.IO сервера -----------------
if __name__ == '__main__':
    rospy.init_node('socketio_server', anonymous=True)

    # Создаем паблишер для топика '/cmd_vel', куда будут отправляться команды движения.
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # Подписываемся на топик '/odom' для получения данных одометрии, используя функцию-обработчик odom_callback.
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Запускаем WSGI-сервер с использованием eventlet на порту 7777.
    eventlet.wsgi.server(eventlet.listen(('0.0.0.0', 7777)), app)
