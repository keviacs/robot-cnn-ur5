#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
import time

# Importar MoveIt Python Interface
try:
    from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
    from moveit_commander.conversions import pose_to_list
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # Configuración de piezas y sus posiciones destino
        self.piece_positions = {
            'screw': {'x': 0.3, 'y': 0.2, 'z': 0.15},      # Posición para tornillos
            'star': {'x': 0.3, 'y': 0.0, 'z': 0.15},       # Posición para estrellas  
            'tee_connector': {'x': 0.3, 'y': -0.2, 'z': 0.15}  # Posición para conectores T
        }
        
        # Posición home del robot
        self.home_position = {'x': 0.5, 'y': 0.0, 'z': 0.3}
        
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
        
        # Suscriptor a coordenadas de objetos
        self.coords_subscription = self.create_subscription(
            Point,
            '/object_coordinates',
            self.coordinates_callback,
            10
        )
        
        # Publisher para estado del sistema
        self.status_publisher = self.create_publisher(
            String,
            '/robot_status',
            10
        )
        
        # Inicializar MoveIt
        self.initialize_moveit()
        
        # Timer para verificar estado del robot
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('🦾 Control Node iniciado - Listo para mover el UR5!')
        if MOVEIT_AVAILABLE:
            self.get_logger().info('✅ MoveIt disponible')
        else:
            self.get_logger().warn('⚠️  MoveIt no disponible - Solo simulando movimientos')

    def initialize_moveit(self):
        """Inicializar comandantes de MoveIt"""
        if not MOVEIT_AVAILABLE:
            self.get_logger().warn('MoveIt no disponible, usando simulación')
            return
            
        try:
            # Inicializar comandantes
            self.robot = RobotCommander()
            self.scene = PlanningSceneInterface()
            self.move_group = MoveGroupCommander("ur_manipulator")  # Grupo del UR5
            
            # Configurar parámetros de planificación
            self.move_group.set_planning_time(10.0)  # 10 segundos máximo de planificación
            self.move_group.set_num_planning_attempts(10)
            self.move_group.set_max_velocity_scaling_factor(0.5)  # Velocidad al 50%
            self.move_group.set_max_acceleration_scaling_factor(0.5)  # Aceleración al 50%
            
            # Obtener información del robot
            planning_frame = self.move_group.get_planning_frame()
            eef_link = self.move_group.get_end_effector_link()
            group_names = self.robot.get_group_names()
            
            self.get_logger().info(f'📋 Marco de planificación: {planning_frame}')
            self.get_logger().info(f'🔗 Enlace del efector final: {eef_link}')
            self.get_logger().info(f'👥 Grupos disponibles: {group_names}')
            
            # Mover a posición home al inicio
            self.move_to_home()
            
        except Exception as e:
            self.get_logger().error(f'❌ Error inicializando MoveIt: {str(e)}')
            self.move_group = None

    def move_to_home(self):
        """Mover el robot a la posición home"""
        if not MOVEIT_AVAILABLE or not self.move_group:
            self.get_logger().info('🏠 Simulando movimiento a HOME')
            return True
            
        try:
            self.get_logger().info('🏠 Moviendo a posición HOME...')
            
            # Crear pose objetivo
            target_pose = Pose()
            target_pose.position.x = self.home_position['x']
            target_pose.position.y = self.home_position['y'] 
            target_pose.position.z = self.home_position['z']
            
            # Orientación hacia abajo (para pick and place)
            target_pose.orientation.x = 1.0
            target_pose.orientation.y = 0.0
            target_pose.orientation.z = 0.0
            target_pose.orientation.w = 0.0
            
            # Planificar y ejecutar
            self.move_group.set_pose_target(target_pose)
            success = self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            
            if success:
                self.get_logger().info('✅ Movimiento a HOME completado')
                return True
            else:
                self.get_logger().warn('⚠️  No se pudo completar movimiento a HOME')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Error moviendo a HOME: {str(e)}')
            return False

    def move_to_position(self, x, y, z):
        """Mover el robot a una posición específica"""
        if self.is_moving:
            self.get_logger().warn('⚠️  Robot ocupado, ignorando comando')
            return False
            
        self.is_moving = True
        
        if not MOVEIT_AVAILABLE or not self.move_group:
            self.get_logger().info(f'🤖 Simulando movimiento a ({x:.3f}, {y:.3f}, {z:.3f})')
            time.sleep(2)  # Simular tiempo de movimiento
            self.is_moving = False
            return True
            
        try:
            self.get_logger().info(f'🎯 Moviendo a posición: ({x:.3f}, {y:.3f}, {z:.3f})')
            
            # Crear pose objetivo
            target_pose = Pose()
            target_pose.position.x = x
            target_pose.position.y = y
            target_pose.position.z = z
            
            # Orientación hacia abajo
            target_pose.orientation.x = 1.0
            target_pose.orientation.y = 0.0
            target_pose.orientation.z = 0.0
            target_pose.orientation.w = 0.0
            
            # Planificar y ejecutar
            self.move_group.set_pose_target(target_pose)
            success = self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            
            self.is_moving = False
            
            if success:
                self.get_logger().info('✅ Movimiento completado exitosamente')
                return True
            else:
                self.get_logger().warn('⚠️  No se pudo completar el movimiento')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Error en movimiento: {str(e)}')
            self.is_moving = False
            return False

    def pick_object(self):
        """Simular acción de agarre"""
        self.get_logger().info('🤏 Ejecutando acción de PICK (agarre)...')
        
        if not MOVEIT_AVAILABLE:
            time.sleep(1)  # Simular tiempo de agarre
            self.get_logger().info('✅ PICK simulado completado')
            return True
        
        # En un sistema real, aquí se controlaría el gripper
        # Por ahora solo bajamos un poco más
        current_pose = self.move_group.get_current_pose().pose
        current_pose.position.z -= 0.05  # Bajar 5cm
        
        self.move_group.set_pose_target(current_pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        if success:
            self.get_logger().info('✅ PICK completado')
        else:
            self.get_logger().warn('⚠️  PICK falló')
            
        return success

    def place_object(self, piece_type):
        """Colocar objeto en posición designada según su tipo"""
        if piece_type not in self.piece_positions:
            self.get_logger().error(f'❌ Tipo de pieza desconocido: {piece_type}')
            return False
            
        target_pos = self.piece_positions[piece_type]
        self.get_logger().info(f'📦 Colocando {piece_type} en posición designada...')
        
        # Mover a posición de colocación
        success = self.move_to_position(
            target_pos['x'], 
            target_pos['y'], 
            target_pos['z']
        )
        
        if success:
            self.get_logger().info(f'📍 {piece_type} colocado en ({target_pos["x"]}, {target_pos["y"]}, {target_pos["z"]})')
            
            # Simular liberación del objeto
            time.sleep(0.5)
            self.get_logger().info('✅ Objeto liberado')
            
        return success

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
                self.execute_pick_and_place_sequence(piece_type, x, y, z)
                
        except Exception as e:
            self.get_logger().error(f'❌ Error procesando detección: {str(e)}')

    def coordinates_callback(self, msg):
        """Callback para coordenadas de objetos (opcional, para debug)"""
        self.last_detection_time = time.time()

    def execute_pick_and_place_sequence(self, piece_type, x, y, z):
        """Ejecutar secuencia completa de pick and place"""
        self.get_logger().info(f'🚀 Iniciando secuencia pick-and-place para {piece_type}')
        
        # 1. Mover al objeto detectado
        if not self.move_to_position(x, y, z + 0.1):  # Agregar offset de seguridad
            self.get_logger().error('❌ Falló movimiento al objeto')
            return
            
        # 2. Ejecutar pick (agarre)
        if not self.pick_object():
            self.get_logger().error('❌ Falló el agarre del objeto')
            return
            
        # 3. Colocar en posición designada
        if not self.place_object(piece_type):
            self.get_logger().error('❌ Falló la colocación del objeto')
            return
            
        # 4. Regresar a home
        self.move_to_home()
        
        self.get_logger().info(f'✅ Secuencia completada exitosamente para {piece_type}')

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
    
    control_node = ControlNode()
    
    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        control_node.get_logger().info('🛑 Control Node detenido por el usuario')
    finally:
        control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

