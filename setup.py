from setuptools import find_packages, setup

package_name = 'grab_sequence'


def _model_files():
    import os
    out = []
    for root, _dirs, files in os.walk('models'):
        if files:
            out.append((f'share/{package_name}/' + root, [os.path.join(root, f) for f in files]))
    return out

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        *_model_files(),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            f'share/{package_name}/launch',
            [
                'launch/grab_sequence.launch.py',
                'launch/grasp_ball.launch.py',
                'launch/probe_reach.launch.py',
                'launch/grasp_trial.launch.py',
            ],
        ),
        # amcl.yaml is a launch input, so it has to be installed rather than
        # read out of the source tree the way the ops scripts used to.
        (f'share/{package_name}/config', ['config/moveit_cpp.yaml',
                                          'config/amcl.yaml']),
    ],
    # Installed so grasp_trial.launch.py can reach them as package executables.
    # bringup.sh reached into the source tree by absolute path, which works from
    # a shell in this workspace and nowhere else.
    scripts=[
        'scripts/scan_self_filter.py',
        'scripts/cmd_vel_guard.py',
        'scripts/nav_grasp_trials.py',
        # nav_grasp_trials imports repeatability_test as a sibling, and
        # collect_trials imports both. They have to land in the same directory
        # or the import fails at runtime with ModuleNotFoundError, which the
        # launch file only surfaces once the trial actually starts.
        'scripts/repeatability_test.py',
        'scripts/collect_trials.py',
        'scripts/ops/seed_amcl.py',
        'scripts/ops/arm_lifecycle.py',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aydan Ling',
    maintainer_email='aydan.ling11@gmail.com',
    description='Autonomous shuttlecock collection on a simulated badminton court.',
    license='Apache-2.0',
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
