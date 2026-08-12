from setuptools import find_packages, setup

package_name = 'grab_sequence'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            f'share/{package_name}/launch',
            [
                'launch/grab_sequence.launch.py',
                'launch/grasp_ball.launch.py',
                'launch/probe_reach.launch.py',
            ],
        ),
        (f'share/{package_name}/config', ['config/moveit_cpp.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aydan-ling',
    maintainer_email='aydan-ling@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'grab_sequence = grab_sequence.grab_sequence:main',
            'ball_detector = grab_sequence.ball_detector:main',
            'grasp_ball = grab_sequence.grasp_ball:main',
            'probe_reach = grab_sequence.probe_reach:main',
        ],
    },
)
