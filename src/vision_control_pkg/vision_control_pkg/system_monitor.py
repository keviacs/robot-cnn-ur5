#!/usr/bin/env python3

# =============================================================================
# ARCHIVO: system_monitor.py
# =============================================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import time

class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        
        # Estado del sistema
        self.camera_active = False
        self.vision_active = False
        self.control_active = False
        self.last_detection = "Ninguna"
        self.detection_count = 0
        
        # Suscriptores para monitorear el sistema
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.detection_sub = self.create_subscription(
            String, '/detected_objects', self.detection_callback, 10)
        self.status_sub = self.create_subscription(
            String, '/robot_status', self.status_callback, 10)
        
        # Timer para mostrar estado
        self.timer = self.create_timer(5.0, self.print_status)
        
        self.get_logger().info('📊 Monitor del Sistema iniciado')

    def image_callback(self, msg):
        self.camera_active = True

    def detection_callback(self, msg):
        self.vision_active = True
        self.detection_count += 1
        data = msg.data.split(',')
        if len(data) >= 1:
            self.last_detection = data[0]

    def status_callback(self, msg):
        self.control_active = True

    def print_status(self):
        """Mostrar estado completo del sistema"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('📊 ESTADO DEL SISTEMA')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'📷 Cámara: {"✅ ACTIVA" if self.camera_active else "❌ INACTIVA"}')
        self.get_logger().info(f'🤖 Visión CNN: {"✅ ACTIVA" if self.vision_active else "❌ INACTIVA"}')
        self.get_logger().info(f'🦾 Control UR5: {"✅ ACTIVO" if self.control_active else "❌ INACTIVO"}')
        self.get_logger().info(f'🔍 Última detección: {self.last_detection}')
        self.get_logger().info(f'📈 Total detecciones: {self.detection_count}')
        self.get_logger().info('=' * 60)
        
        # Reset flags para próximo ciclo
        self.camera_active = False
        self.vision_active = False
        self.control_active = False

def main(args=None):
    rclpy.init(args=args)
    monitor = SystemMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('🛑 Monitor detenido')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
