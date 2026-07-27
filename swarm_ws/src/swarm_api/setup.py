from setuptools import setup

package_name = 'swarm_api'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SwarmCore',
    maintainer_email='dev@lcw.local',
    description='SwarmCore 集群控制框架：多机并行的位置/速度控制 API',
    license='BSD-3-Clause',
)
