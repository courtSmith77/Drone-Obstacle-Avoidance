from setuptools import setup

package_name = 'fly_drone'
mod = "fly_drone/modules"
setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=[package_name, mod],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['config/arrow_best.pt']),
        ('share/' + package_name, ['config/detect_best.pt']),
        ('share/' + package_name, ['launch/visual.launch.xml']),
        ('share/' + package_name, ['config/view_live_feed.rviz']),
        ('share/' + package_name, ['config/special_best.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='csmith',
    maintainer_email='courtney77smith77@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone = fly_drone.drone:main'
        ],
    },
)
