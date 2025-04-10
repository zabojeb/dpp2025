#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import socketio
import eventlet
import eventlet.wsgi
import struct
import can

# Глобальные переменные для ROS и управления сервоприводами
pub = None            # Паблишер для cmd_vel
latest_odom = None    # Последнее сообщение одометрии
servo_values = [90.0] * 7  # 7 значений (первые 6 — углы, 7-й — скорость)

# Функция-обработчик сообщений одометрии ROS
def odom_callback(msg):
    global latest_odom
    latest_odom = msg

# Преобразование сообщения Odometry в словарь для отправки клиенту
def odom_to_dict(msg):
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

# Функция публикации команды cmd_vel в ROS
def publish_cmd_vel(data):
    twist = Twist()
    try:
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
    pub.publish(twist)
    rospy.loginfo("Опубликовано: {}".format(data))
    return True, "Команда опубликована"

# Функция отправки значений сервоприводов через CAN-шину
def send_servo_values():
    data = struct.pack('7f', *servo_values)
    msg = can.Message(
        arbitration_id=0x700,    # Идентификатор CAN-сообщения
        data=data,
        is_extended_id=False,    # Стандартный 11-битный ID
        is_fd=True               # Флаг CAN-FD
    )
    try:
        with can.interface.Bus(channel='can0', bustype='socketcan', fd=True) as bus:
            bus.send(msg)
            rospy.loginfo("Отправлено: {}".format(servo_values))
    except can.CanError as e:
        rospy.logerr("Ошибка отправки CAN-сообщения: {}".format(e))

# Обновление значения конкретного сервопривода
def update_servo(servo_id, value):
    if servo_id < 0 or servo_id >= len(servo_values):
        return False
    servo_values[servo_id] = float(value)
    send_servo_values()
    return True


# Создание Socket.IO сервера с поддержкой CORS
sio = socketio.Server(cors_allowed_origins='*')
# Оборачиваем Socket.IO сервер в WSGI-приложение
app = socketio.WSGIApp(sio)

# --- Обработчики Socket.IO событий ---

@sio.event
def connect(sid, environ):
    rospy.loginfo("Клиент подключился: {}".format(sid))
    # При подключении отправляем клиенту текущие значения сервоприводов
    sio.emit("servo_update", {"servo_values": servo_values}, room=sid)


@sio.event
def disconnect(sid):
    rospy.loginfo("Клиент отключился: {}".format(sid))

# Обработчик изменения значений сервоприводов
@sio.event
def set_servo(sid, data):
    servo_id = data.get("servo_id")
    value = data.get("value")
    if servo_id is None or value is None:
        sio.emit("error", {"error": "Missing servo_id or value"}, room=sid)
        return
    if not update_servo(servo_id, value):
        sio.emit("error", {"error": "Invalid servo_id"}, room=sid)
        return

    # Рассылаем обновлённые значения всем подключённым клиентам
    sio.emit("servo_update", {"servo_values": servo_values})

# Обработчик получения команды движения (cmd_vel) от клиента
@sio.event
def cmd_vel(sid, data):
    success, message = publish_cmd_vel(data)
    if success:
        sio.emit("cmd_vel_response", {
                 "status": "success", "message": message}, room=sid)
    else:
        sio.emit("cmd_vel_response", {
                 "status": "error", "message": message}, room=sid)

# Обработчик запроса данных одометрии
@sio.event
def get_odom(sid):
    if latest_odom is None:
        sio.emit("odom_response", {
                 "status": "error", "message": "Данные одометрии недоступны"}, room=sid)
    else:
        odom_data = odom_to_dict(latest_odom)
        sio.emit("odom_response", odom_data, room=sid)


# --- Инициализация ROS и запуск сервера ---
if __name__ == '__main__':
    rospy.init_node('socketio_server', anonymous=True)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    eventlet.wsgi.server(eventlet.listen(('0.0.0.0', 7777)), app)
