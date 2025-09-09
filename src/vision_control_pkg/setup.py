from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision_control_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Modelos de IA
        (os.path.join('share', package_name, 'models'),
            glob(os.path.join('models', '*.keras'))),
        # Imágenes de prueba
        (os.path.join('share', package_name, 'test_images'),
            glob(os.path.join('test_images', '*.jpg'))),
        (os.path.join('share', package_name, 'test_images'),
            glob(os.path.join('test_images', '*.png'))),
    ],
    install_requires=[
        'setuptools',
        'tensorflow>=2.8.0',
        'opencv-python>=4.5.0',
        'numpy>=1.19.0',
        'Pillow>=8.0.0',
    ],
    zip_safe=True,
    maintainer='Kevin',
    maintainer_email='kevin@robotics.com',
    description='Sistema completo de visión CNN + control de brazo robótico UR5',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = vision_control_pkg.vision_node:main',
            'control_node = vision_control_pkg.control_node:main',
            'camera_publisher = vision_control_pkg.camera_publisher:main',
            'system_monitor = vision_control_pkg.system_monitor:main',
            'control_node_simple = vision_control_pkg.control_node_simple:main',
        ],
    },
)


