from setuptools import find_packages, setup

package_name = 'nodos_varios'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elio',
    maintainer_email='elio@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_test = nodos_varios.node_test:main',
            'node_w = nodos_varios.node_window:main',
            'node_param = nodos_varios.primer_parametros:main',
            'node_t_matrix = nodos_varios.node_translation:main',
            'node_teleop = nodos_varios.teleop_keyboard_robot:main',
            'node_prueba_a = nodos_varios.test_save_node:main',
            'node_lectura_vel = nodos_varios.test_lectura_vel:main',
            'nodo_transformacion = nodos_varios.simple_tf_kinematics:main',
            'simple_service_s = nodos_varios.simple_servide:main',
            'simple_service_client = nodos_varios.simple_client:main'
        ],
    },
)
