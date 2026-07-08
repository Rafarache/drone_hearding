from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cow_pkg'

def get_model_data_files():
    data_files = []
    for root, dirs, files in os.walk('models'):
        if files:
            # Caminho de instalação preservando a estrutura
            install_dir = os.path.join('share', package_name, root)
            # Caminhos completos dos arquivos
            file_paths = [os.path.join(root, f) for f in files]
            data_files.append((install_dir, file_paths))
    return data_files



setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ] + get_model_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='feettree.1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'repeller = cow_pkg.repeller_node:main',
        ],
    },
)
