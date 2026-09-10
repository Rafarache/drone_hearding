from setuptools import find_packages, setup

package_name = 'yolo_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    include_package_data=True,
    package_data={
        # include checkpoint files under the depth_pkg/checkpoints directory
        'yolo_pkg': ['depth_pkg/checkpoints/*']
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='feettree.1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_subscriber = yolo_pkg.yolo_subscriber:main',
        ],
    },
)
