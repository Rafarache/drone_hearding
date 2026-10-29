from setuptools import setup, find_packages
setup(name='simulation_test_pkg', version='1.0.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/simulation_test_pkg']),
                  ('share/simulation_test_pkg', ['package.xml', 'requirements.txt', 'requirements.lock', 'README.md'])],
      install_requires=['setuptools'], zip_safe=False,
      maintainer='Rafael', maintainer_email='rafael@example.com',
      description='Local experiment runner, trajectory recorder, and replay dashboard', license='MIT',
      entry_points={'console_scripts': ['test_lab = simulation_test_pkg.server:main',
                                        'run_batch = simulation_test_pkg.cli:main']})
