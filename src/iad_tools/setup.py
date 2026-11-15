from setuptools import setup

setup(
    name='iad_tools',
    version='0.2.0',
    packages=['iad_tools'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/iad_tools']),
                ('share/iad_tools', ['package.xml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iad',
    description='HUD overlays and metrics logging for the demos.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['hud = iad_tools.hud:main', 'metrics_logger = iad_tools.metrics_logger:main',
                                  'mission = iad_tools.mission:main', 'vehicle_marker = iad_tools.vehicle_marker:main',
                                  'pcd_map = iad_tools.pcd_map:main']},
)
