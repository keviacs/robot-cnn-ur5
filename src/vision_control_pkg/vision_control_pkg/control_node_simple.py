#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import time
import math

class SimpleControlNode(Node):
    def __init__(self):
        super().__init__('simple_control_node')
        
        # Configuración de piezas y sus posiciones destino (en ángulos de articulaciones)
        self.piece_positions = {
            'screw': {
                'shoulder_pan_joint': 0.5,      # Girar hacia la derecha
                'shoulder_lift_joint': -0.8,    # Levantar brazo
                'elbow_joint': 1.2,             # Doblar codo
                'wrist_1_joint': -0.4,          # Orientar muñeca
                'wrist_2_joint': 1.57,          # 90 grados
                'wrist_3_joint': 0.0
            },
            'star': {
                'shoulder_pan_joint': 0.0,      # Posición central
                'shoulder_lift_joint': -1.0,    
                'elbow_joint': 1.5,
                'wrist_1_joint': -0.5,
                'wrist_2_joint': 1.57,
                'wrist_3_joint': 0.0
            },
            'tee_connector': {
                'shoulder_pan_joint': -0.5,     # Girar hacia la izquierda
                'shoulder_lift_joint': -0.6,
                'elbow_joint': 1.0,
                'wrist_1_joint': -0.4,
                'wrist_2_joint': 1.57,
                'wrist_3_joint': 0.0
            }
        }
        
        # Posición home del robot
        self.home_position = {
            'shoulder_pan_joint': 0.0,
            'shoulder_lift_joint': -1.57,  # Brazo hacia arriba
            'elbow_joint': 0.0,
            'wrist_1_joint': -1.57,
            'wrist_2_joint': 0.0,
            'wrist_3_joint': 0.0
        }
        
        # Posición actual del robot
        self.current_position = self.home_position.copy()
        
        # Estado del sistema
        self.is_moving = False
        self.last_detection_time = time.time()
        
        # Suscriptor a detecciones de objetos
        self.detection_subscription = self.create_subscription(
            String,
            '/detected_objects',
            self.detection_callback,
            10
        )
        
        # Publisher para joint states
        self.joint_state_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Publisher para estado del sistema
        self.status_publisher = self.create_publisher(
            String,
            '/robot_status',
            10
        )
        
        # Timer para publicar joint states continuamente
        self.joint_timer = self.create_timer(0.1, self.publish_joint_states)
        
        # Timer para verificar estado del robot
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('🦾 Simple Control Node iniciado - Control directo del UR5!')
        self.get_logger().info('✅ Modo directo activado - Robot se moverá inmediatamente')
        
        # Mover a posición home al inicio
        self.move_to_home()

    def publish_joint_states(self):
        """Publicar estados actuales de las articulaciones"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ''
        
        # Nombres de las articulaciones del UR5
        joint_state.name = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Posiciones actuales
        joint_state.position = [
            self.current_position['shoulder_pan_joint'],
            self.current_position['shoulder_lift_joint'],
            self.current_position['elbow_joint'],
            self.current_position['wrist_1_joint'],
            self.current_position['wrist_2_joint'],
            self.current_position['wrist_3_joint']
        ]
        
        # Velocidades (todas cero para posición estática)
        joint_state.velocity = [0.0] * 6
        joint_state.effort = [0.0] * 6
        
        self.joint_state_publisher.publish(joint_state)

    def move_to_home(self):
        """Mover el robot a la posición home"""
        self.get_logger().info('🏠 Moviendo a posición HOME...')
        self.move_to_joint_positions(self.home_position)
        self.get_logger().info('✅ Movimiento a HOME completado')

    def move_to_joint_positions(self, target_positions):
        """Mover el robot a posiciones específicas de articulaciones con interpolación suave"""
        if self.is_moving:
            self.get_logger().warn('⚠️  Robot ocupado, ignorando comando')
            return False
            
        self.is_moving = True
        
        # Guardar posiciones inicial y objetivo
        start_positions = self.current_position.copy()
        
        # Interpolación suave en 50 pasos (5 segundos a 10Hz)
        steps = 50
        for step in range(steps + 1):
            t = step / steps  # Factor de interpolación de 0 a 1
            
            # Interpolación lineal para cada articulación
            for joint_name in target_positions:
                start_angle = start_positions[joint_name]
                target_angle = target_positions[joint_name]
                self.current_position[joint_name] = start_angle + t * (target_angle - start_angle)
            
            # Esperar para crear movimiento suave
            time.sleep(0.1)
        
        self.is_moving = False
        return True

    def detection_callback(self, msg):
        """Callback para procesar detecciones de objetos"""
        try:
            # Parsear mensaje: "clase,confianza,x,y,z"
            data = msg.data.split(',')
            if len(data) != 5:
                self.get_logger().warn(f'⚠️  Formato de mensaje inválido: {msg.data}')
                return
                
            piece_type = data[0]
            confidence = float(data[1])
            x, y, z = float(data[2]), float(data[3]), float(data[4])
            
            self.get_logger().info(
                f'🔍 Recibida detección: {piece_type} '
                f'(confianza: {confidence:.1%}) '
                f'en ({x:.3f}, {y:.3f}, {z:.3f})'
            )
            
            # Solo procesar si la confianza es alta y el robot no está ocupado
            if confidence >= 0.7 and not self.is_moving:
                self.execute_pick_and_place_sequence(piece_type)
                
        except Exception as e:
            self.get_logger().error(f'❌ Error procesando detección: {str(e)}')

    def execute_pick_and_place_sequence(self, piece_type):
        """Ejecutar secuencia completa de pick and place"""
        if piece_type not in self.piece_positions:
            self.get_logger().error(f'❌ Tipo de pieza desconocido: {piece_type}')
            return
            
        self.get_logger().info(f'🚀 Iniciando secuencia de movimiento para {piece_type}')
        
        # 1. Mover a posición específica de la pieza
        target_position = self.piece_positions[piece_type]
        self.get_logger().info(f'🎯 Moviendo a posición de {piece_type}')
        
        if self.move_to_joint_positions(target_position):
            self.get_logger().info(f'✅ Robot movido a posición de {piece_type}')
            
            # 2. Simular pick (pausa)
            time.sleep(1.0)
            self.get_logger().info('🤏 Simulando agarre...')
            
            # 3. Regresar a home
            time.sleep(1.0)
            self.get_logger().info('🏠 Regresando a HOME')
            self.move_to_home()
            
            self.get_logger().info(f'✅ Secuencia completada para {piece_type}')
        else:
            self.get_logger().error(f'❌ Falló movimiento para {piece_type}')

    def publish_status(self):
        """Publicar estado actual del robot"""
        status_msg = String()
        
        if self.is_moving:
            status_msg.data = "MOVING"
        elif time.time() - self.last_detection_time < 5.0:
            status_msg.data = "DETECTING"  
        else:
            status_msg.data = "IDLE"
            
        self.status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    control_node = SimpleControlNode()
    
    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        control_node.get_logger().info('🛑 Simple Control Node detenido por el usuario')
    finally:
        control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
