from setuptools import setup

package_name = 'indention_test'

setup(
    name=package_name,
    version='0.0.0',
    package_dir={f'{package_name}_utils' : f'{package_name}/{package_name}_utils'}, 
    packages=[package_name, f'{package_name}_utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lab-4f',
    maintainer_email='lab-4f@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tester = ' + package_name + '.indention_tester:main',
            'record_csv = ' + package_name + '.record_csv:main',
        ],
    },
)
