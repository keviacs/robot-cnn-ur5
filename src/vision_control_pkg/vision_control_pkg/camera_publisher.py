#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time
from datetime import datetime

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Bridge para conversión de imágenes
        self.bridge = CvBridge()
        
        # Publishers
        self.image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        # Configuración de la cámara virtual
        self.frame_id = "camera_link"
        self.width = 640
        self.height = 480
        self.fps = 10  # 10 FPS
        
        # Modo de operación
        self.declare_parameter('camera_mode', 'test_images')  # 'test_images', 'webcam', 'generated'
        self.camera_mode = self.get_parameter('camera_mode').value
        
        # Rutas de imágenes de prueba
        self.test_images_dir = 'test_images/'
        self.current_image_index = 0
        self.test_images = []
        
        # Para webcam
        self.cap = None
        
        # Cargar imágenes de prueba o inicializar webcam
        self.initialize_camera_source()
        
        # Timer para publicar imágenes
        self.timer = self.create_timer(1.0 / self.fps, self.publish_image)
        
        self.get_logger().info(f'Cámara virtual iniciada en modo: {self.camera_mode}')
        self.get_logger().info(f'Resolución: {self.width}x{self.height} @ {self.fps} FPS')

    def initialize_camera_source(self):
        """Inicializar la fuente de imágenes según el modo"""
        if self.camera_mode == 'test_images':
            self.load_test_images()
        elif self.camera_mode == 'webcam':
            self.initialize_webcam()
        elif self.camera_mode == 'generated':
            self.get_logger().info('Generando imágenes sintéticas')
        else:
            self.get_logger().warn(f'Modo desconocido: {self.camera_mode}, usando imágenes generadas')
            self.camera_mode = 'generated'

    def load_test_images(self):
        """Cargar imágenes de prueba desde el directorio"""
        # Buscar directorio de imágenes de prueba
        search_paths = [
            self.test_images_dir,
            'src/vision_control_pkg/test_images/',
            '../test_images/',
            os.path.expanduser('~/robot_arm_ws/src/vision_control_pkg/test_images/')
        ]
        
        images_found = False
        for path in search_paths:
            if os.path.exists(path):
                # Buscar archivos de imagen
                for filename in os.listdir(path):
                    if filename.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp')):
                        full_path = os.path.join(path, filename)
                        self.test_images.append(full_path)
                        images_found = True
                
                if images_found:
                    self.get_logger().info(f'Imágenes cargadas desde: {path}')
                    self.get_logger().info(f'{len(self.test_images)} imágenes encontradas')
                    for img_path in self.test_images:
                        self.get_logger().info(f'   - {os.path.basename(img_path)}')
                    break
        
        if not images_found:
            self.get_logger().warn('No se encontraron imágenes de prueba')
            self.get_logger().info('Creando imágenes sintéticas de ejemplo...')
            self.create_sample_images()

    def create_sample_images(self):
        """Crear imágenes sintéticas de ejemplo"""
        # Crear directorio si no existe
        os.makedirs('test_images', exist_ok=True)
        
        # Colores para cada tipo de pieza
        piece_colors = {
            'screw': (100, 100, 100),    # Gris para tornillo
            'star': (0, 255, 255),       # Amarillo para estrella
            'tee_connector': (255, 100, 100)  # Azul claro para conector T
        }
        
        for piece_name, color in piece_colors.items():
            # Crear imagen base
            img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            img.fill(50)  # Fondo gris oscuro
            
            # Dibujar forma según el tipo de pieza
            center_x, center_y = self.width // 2, self.height // 2
            
            if piece_name == 'screw':
                # Dibujar tornillo como rectángulo con círculo
                cv2.rectangle(img, (center_x-20, center_y-60), (center_x+20, center_y+60), color, -1)
                cv2.circle(img, (center_x, center_y-40), 25, color, -1)
                cv2.circle(img, (center_x, center_y-40), 8, (0, 0, 0), -1)  # Agujero
                
            elif piece_name == 'star':
                # Dibujar estrella
                points = []
                for i in range(10):
                    angle = i * np.pi / 5
                    radius = 50 if i % 2 == 0 else 25
                    x = int(center_x + radius * np.cos(angle))
                    y = int(center_y + radius * np.sin(angle))
                    points.append((x, y))
                points = np.array(points, np.int32)
                cv2.fillPoly(img, [points], color)
                
            elif piece_name == 'tee_connector':
                # Dibujar conector T
                cv2.rectangle(img, (center_x-60, center_y-10), (center_x+60, center_y+10), color, -1)  # Horizontal
                cv2.rectangle(img, (center_x-10, center_y-40), (center_x+10, center_y+40), color, -1)  # Vertical
            
            # Agregar algo de ruido para hacer la imagen más realista
            noise = np.random.randint(-20, 20, img.shape, dtype=np.int16)
            img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
            
            # Guardar imagen
            filename = f'test_images/{piece_name}_test.jpg'
            cv2.imwrite(filename, img)
            self.test_images.append(filename)
            
        self.get_logger().info(f'{len(self.test_images)} imágenes sintéticas creadas')

    def initialize_webcam(self):
        """Inicializar webcam si está disponible"""
        try:
            self.cap = cv2.VideoCapture(0)  # Cámara por defecto
            if not self.cap.isOpened():
                self.get_logger().warn('No se pudo abrir la webcam, usando imágenes generadas')
                self.camera_mode = 'generated'
                return
                
            # Configurar resolución
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            self.get_logger().info('Webcam inicializada correctamente')
            
        except Exception as e:
            self.get_logger().error(f'Error inicializando webcam: {str(e)}')
            self.camera_mode = 'generated'

    def generate_synthetic_image(self):
        """Generar imagen sintética aleatoria"""
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        img.fill(30)  # Fondo gris muy oscuro
        
        # Agregar algunos objetos aleatorios
        num_objects = np.random.randint(1, 3)
        piece_types = ['screw', 'star', 'tee_connector']
        colors = [(100, 100, 100), (0, 255, 255), (255, 100, 100)]
        
        for _ in range(num_objects):
            # Posición aleatoria
            x = np.random.randint(100, self.width - 100)
            y = np.random.randint(100, self.height - 100)
            
            # Tipo y color aleatorio
            piece_idx = np.random.randint(0, len(piece_types))
            color = colors[piece_idx]
            
            # Dibujar forma simple
            if piece_idx == 0:  # screw
                cv2.rectangle(img, (x-15, y-40), (x+15, y+40), color, -1)
                cv2.circle(img, (x, y-25), 20, color, -1)
            elif piece_idx == 1:  # star
                cv2.circle(img, (x, y), 30, color, -1)
            else:  # tee_connector
                cv2.rectangle(img, (x-40, y-8), (x+40, y+8), color, -1)
                cv2.rectangle(img, (x-8, y-30), (x+8, y+30), color, -1)
        
        # Agregar ruido
        noise = np.random.randint(-15, 15, img.shape, dtype=np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        
        return img

    def get_current_image(self):
        """Obtener imagen actual según el modo configurado"""
        if self.camera_mode == 'test_images' and self.test_images:
            # Ciclar entre imágenes de prueba
            image_path = self.test_images[self.current_image_index]
            img = cv2.imread(image_path)
            
            if img is None:
                self.get_logger().warn(f'No se pudo cargar imagen: {image_path}')
                return self.generate_synthetic_image()
            
            # Redimensionar si es necesario
            img = cv2.resize(img, (self.width, self.height))
            
            # Avanzar al siguiente índice cada 15 segundos
            if int(time.time()) % 15 == 0:
                self.current_image_index = (self.current_image_index + 1) % len(self.test_images)
            
            return img
            
        elif self.camera_mode == 'webcam' and self.cap:
            ret, frame = self.cap.read()
            if ret:
                return cv2.resize(frame, (self.width, self.height))
            else:
                self.get_logger().warn('Error leyendo de webcam')
                return self.generate_synthetic_image()
                
        else:  # generated mode
            return self.generate_synthetic_image()

    def create_camera_info_msg(self, header):
        """Crear mensaje CameraInfo"""
        camera_info = CameraInfo()
        camera_info.header = header
        camera_info.width = self.width
        camera_info.height = self.height
        
        # Parámetros de cámara simulados (focal length, etc.)
        fx = fy = 500.0  # Focal length en píxeles
        cx = self.width / 2.0   # Centro óptico x
        cy = self.height / 2.0  # Centro óptico y
        
        camera_info.k = [fx, 0.0, cx,
                        0.0, fy, cy,
                        0.0, 0.0, 1.0]
        
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # Sin distorsión
        
        camera_info.r = [1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0]
        
        camera_info.p = [fx, 0.0, cx, 0.0,
                        0.0, fy, cy, 0.0,
                        0.0, 0.0, 1.0, 0.0]
        
        return camera_info

    def publish_image(self):
        """Callback del timer para publicar imágenes"""
        try:
            # Obtener imagen actual
            cv_image = self.get_current_image()
            
            # Crear header con timestamp
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.frame_id
            
            # Convertir imagen OpenCV a mensaje ROS
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            image_msg.header = header
            
            # Crear mensaje CameraInfo
            camera_info_msg = self.create_camera_info_msg(header)
            
            # Publicar ambos mensajes
            self.image_publisher.publish(image_msg)
            self.camera_info_publisher.publish(camera_info_msg)
            
            # Log periódico (cada 5 segundos)
            if int(time.time()) % 5 == 0:
                mode_info = ""
                if self.camera_mode == 'test_images':
                    current_img = os.path.basename(self.test_images[self.current_image_index]) if self.test_images else "ninguna"
                    mode_info = f" (imagen actual: {current_img})"
                
                self.get_logger().info(f'Publicando imagen ({self.camera_mode}){mode_info}')
                
        except Exception as e:
            self.get_logger().error(f'Error publicando imagen: {str(e)}')

    def __del__(self):
        """Limpiar recursos al destruir el nodo"""
        if self.cap:
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    
    camera_publisher = CameraPublisher()
    
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        camera_publisher.get_logger().info('Cámara virtual detenida por el usuario')
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

