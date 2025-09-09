#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String, Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tf
import os
from datetime import datetime
import time

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # Configuración de la CNN
        self.classes = ['screw', 'star', 'tee_connector']
        self.IMG_WIDTH = 224
        self.IMG_HEIGHT = 224
        
        # Bridge para convertir imágenes ROS <-> OpenCV
        self.bridge = CvBridge()
        
        # Cargar modelo CNN
        self.load_model()
        
        # Suscriptor a imágenes de la cámara
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.detection_publisher = self.create_publisher(
            String,
            '/detected_objects',
            10
        )
        
        self.coords_publisher = self.create_publisher(
            Point,
            '/object_coordinates',
            10
        )
        
        # Publisher para imagen procesada (opcional, para debug)
        self.processed_image_publisher = self.create_publisher(
            Image,
            '/processed_image',
            10
        )
        
        self.get_logger().info('🤖 Vision Node iniciado - Listo para detectar piezas!')
        # Control de frecuencia de detecciones
        self.last_detection_time = 0
        self.detection_cooldown = 5.0  # 5 segundos entre detecciones
        self.get_logger().info(f'📋 Clases configuradas: {self.classes}')

    def load_model(self):
        """Cargar el modelo CNN entrenado"""
        try:
            # Buscar el modelo en diferentes ubicaciones
            model_paths = [
                'models/mi_modelo_ia.keras',
                '../models/mi_modelo_ia.keras',
                'src/vision_control_pkg/models/mi_modelo_ia.keras',
                os.path.expanduser('~/robot_arm_ws/src/vision_control_pkg/models/mi_modelo_ia.keras')
            ]
            
            model_loaded = False
            for path in model_paths:
                if os.path.exists(path):
                    self.model = tf.keras.models.load_model(path)
                    self.get_logger().info(f'✅ Modelo cargado desde: {path}')
                    model_loaded = True
                    break
            
            if not model_loaded:
                self.get_logger().error('❌ No se pudo encontrar mi_modelo_ia.keras')
                self.get_logger().error('📁 Ubicaciones buscadas:')
                for path in model_paths:
                    self.get_logger().error(f'   - {path}')
                return
                
        except Exception as e:
            self.get_logger().error(f'❌ Error cargando modelo: {str(e)}')

    def preprocess_image(self, cv_image):
        """Preprocesar imagen para la CNN"""
        # Redimensionar imagen
        resized = cv2.resize(cv_image, (self.IMG_WIDTH, self.IMG_HEIGHT))
        
        # Normalizar píxeles (0-1)
        normalized = resized.astype('float32') / 255.0
        
        # Agregar dimensión batch
        batch_image = np.expand_dims(normalized, axis=0)
        
        return batch_image

    def detect_objects(self, cv_image):
        """Detectar objetos usando la CNN"""
        try:
            # Preprocesar imagen
            preprocessed = self.preprocess_image(cv_image)
            
            # Hacer predicción
            predictions = self.model.predict(preprocessed, verbose=0)
            predicted_class_idx = np.argmax(predictions[0])
            confidence = float(predictions[0][predicted_class_idx])
            
            # Obtener clase predicha
            predicted_class = self.classes[predicted_class_idx]
            
            return predicted_class, confidence, predictions[0]
            
        except Exception as e:
            self.get_logger().error(f'❌ Error en detección: {str(e)}')
            return None, 0.0, None

    def get_object_coordinates(self, cv_image, detected_class):
        """Calcular coordenadas aproximadas del objeto (centro de imagen por ahora)"""
        height, width = cv_image.shape[:2]
        
        # Por simplicidad, usar centro de imagen
        # En un sistema real, se usaría detección de contornos o bounding boxes
        center_x = width // 2
        center_y = height // 2
        
        # Convertir píxeles a coordenadas del mundo (valores simulados)
        # En un sistema real, se necesitaría calibración de cámara
        world_x = (center_x - width//2) * 0.001  # Escala aproximada
        world_y = (center_y - height//2) * 0.001
        world_z = 0.1  # Altura de trabajo fija
        
        return world_x, world_y, world_z

    def image_callback(self, msg):
        """Callback principal para procesar imágenes"""
        try:
            # Convertir imagen ROS a OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Detectar objetos
            detected_class, confidence, all_predictions = self.detect_objects(cv_image)
            
             
            current_time = time.time()
            if detected_class and confidence > 0.7 and (current_time - self.last_detection_time) > self.detection_cooldown:
                self.last_detection_time = current_time
                # Calcular coordenadas
                x, y, z = self.get_object_coordinates(cv_image, detected_class)
                
                # Crear mensaje de detección
                detection_msg = String()
                detection_msg.data = f"{detected_class},{confidence:.3f},{x:.3f},{y:.3f},{z:.3f}"
                
                # Crear mensaje de coordenadas
                coords_msg = Point()
                coords_msg.x = float(x)
                coords_msg.y = float(y)
                coords_msg.z = float(z)
                
                # Publicar resultados
                self.detection_publisher.publish(detection_msg)
                self.coords_publisher.publish(coords_msg)
                
                # Log de detección
                self.get_logger().info(
                    f'🔍 Detectado: {detected_class} '
                    f'(confianza: {confidence:.1%}) '
                    f'en ({x:.3f}, {y:.3f}, {z:.3f})'
                )
                
                # Dibujar información en la imagen (para debug)
                self.draw_detection_info(cv_image, detected_class, confidence, all_predictions)
                
            else:
                # Log cuando no hay detección válida
                if detected_class:
                    self.get_logger().info(f'⚠️  Detección con baja confianza: {detected_class} ({confidence:.1%})')
            
            # Publicar imagen procesada (opcional)
            try:
                processed_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
                processed_msg.header = msg.header
                self.processed_image_publisher.publish(processed_msg)
            except Exception as e:
                self.get_logger().warn(f'No se pudo publicar imagen procesada: {e}')
                
        except Exception as e:
            self.get_logger().error(f'❌ Error procesando imagen: {str(e)}')

    def draw_detection_info(self, image, detected_class, confidence, all_predictions):
        """Dibujar información de detección en la imagen"""
        height, width = image.shape[:2]
        
        # Dibujar círculo en el centro (posición detectada)
        center = (width//2, height//2)
        cv2.circle(image, center, 20, (0, 255, 0), 3)
        
        # Información de detección
        info_text = f"{detected_class}: {confidence:.1%}"
        cv2.putText(image, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.8, (0, 255, 0), 2)
        
        # Mostrar predicciones de todas las clases
        y_offset = 60
        for i, class_name in enumerate(self.classes):
            pred_text = f"{class_name}: {all_predictions[i]:.1%}"
            color = (0, 255, 0) if i == np.argmax(all_predictions) else (255, 255, 255)
            cv2.putText(image, pred_text, (10, y_offset + i*25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

def main(args=None):
    rclpy.init(args=args)
    
    vision_node = VisionNode()
    
    try:
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        vision_node.get_logger().info('🛑 Vision Node detenido por el usuario')
    finally:
        vision_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

