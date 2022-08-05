from setuptools import setup

package_name = 'omnino_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eduardo Henrique',
    maintainer_email='ehfs@ic.ufal.br',
    description='TODO: Package description',
    license='Apacha License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'omnino_driver_node = omnino_driver.omnino_driver_node:main'
        ],
    },
)