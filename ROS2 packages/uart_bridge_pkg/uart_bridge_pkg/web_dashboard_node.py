#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from flask import Flask, send_from_directory
from flask_socketio import SocketIO
import threading
import os
import time
from ament_index_python.packages import get_package_share_directory

app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading')

try:
    package_share_directory = get_package_share_directory('uart_bridge_pkg')
    static_dir = os.path.join(package_share_directory, 'static')
except Exception as e:
    # Резервный путь для локального тестирования без установки
    static_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'static')

class WebDashboardNode(Node):
    def __init__(self):
        super().__init__('web_dashboard_node')
        
        # Подписка на сырые энкодеры
        self.create_subscription(Int32MultiArray, '/encoders_raw', self.encoders_cb, 10)
        
        # Клиент сервиса для установки параметров в uart_bridge_node
        self.param_client = self.create_client(SetParameters, '/uart_bridge_node/set_parameters')

    def encoders_cb(self, msg):
        # Отправка через socketio
        socketio.emit('encoder_data', list(msg.data))

    def update_pid_params(self, motor_id, kp, ki, kd, kff, min_pwm, kp_sync):
        # Ожидание доступности сервиса
        if not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service /uart_bridge_node/set_parameters not available')
            return
        
        req = SetParameters.Request()
        
        p_motor = Parameter()
        p_motor.name = 'motor_id'
        p_motor.value.type = ParameterType.PARAMETER_INTEGER
        p_motor.value.integer_value = int(motor_id)
        
        p_kp = Parameter()
        p_kp.name = 'kp'
        p_kp.value.type = ParameterType.PARAMETER_DOUBLE
        p_kp.value.double_value = float(kp)
        
        p_ki = Parameter()
        p_ki.name = 'ki'
        p_ki.value.type = ParameterType.PARAMETER_DOUBLE
        p_ki.value.double_value = float(ki)
        
        p_kd = Parameter()
        p_kd.name = 'kd'
        p_kd.value.type = ParameterType.PARAMETER_DOUBLE
        p_kd.value.double_value = float(kd)
        
        p_kff = Parameter()
        p_kff.name = 'kff'
        p_kff.value.type = ParameterType.PARAMETER_DOUBLE
        p_kff.value.double_value = float(kff)
        
        p_min_pwm = Parameter()
        p_min_pwm.name = 'min_pwm'
        p_min_pwm.value.type = ParameterType.PARAMETER_INTEGER
        p_min_pwm.value.integer_value = int(min_pwm)
        
        p_kp_sync = Parameter()
        p_kp_sync.name = 'kp_sync'
        p_kp_sync.value.type = ParameterType.PARAMETER_DOUBLE
        p_kp_sync.value.double_value = float(kp_sync)
        
        req.parameters = [p_motor, p_kp, p_ki, p_kd, p_kff, p_min_pwm, p_kp_sync]
        
        future = self.param_client.call_async(req)
        future.add_done_callback(self.param_response_cb)

    def param_response_cb(self, future):
        try:
            response = future.result()
            self.get_logger().info('Parameters updated successfully via Web')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

ros_node = None

@app.route('/')
def index():
    return send_from_directory(static_dir, 'index.html')

@socketio.on('update_pid')
def handle_update_pid(data):
    if ros_node:
        motor_id = data.get('motor_id', 0)
        kp = data.get('kp', 0.0)
        ki = data.get('ki', 0.0)
        kd = data.get('kd', 0.0)
        kff = data.get('kff', 0.0)
        min_pwm = data.get('min_pwm', 0)
        kp_sync = data.get('kp_sync', 0.0)
        ros_node.get_logger().info(f"Received web update: motor={motor_id}, Kp={kp}, Ki={ki}, Kd={kd}, Kff={kff}, MinPWM={min_pwm}, KpSync={kp_sync}")
        ros_node.update_pid_params(motor_id, kp, ki, kd, kff, min_pwm, kp_sync)

def spin_ros():
    rclpy.spin(ros_node)

def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = WebDashboardNode()
    
    # Запуск ROS в отдельном потоке
    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()
    
    # Запуск Flask в главном потоке
    try:
        ros_node.get_logger().info("Starting Flask-SocketIO server on 0.0.0.0:5000")
        socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        ros_thread.join()

if __name__ == '__main__':
    main()
