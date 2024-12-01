from setuptools import find_packages, setup

package_name = 'mmi_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2vm',
    maintainer_email='ros2vm@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rotary_listener = mmi_package.rotary_subscriber:main',
            'rotary_btn_listener = mmi_package.rotary_btn_subscriber:main',
            'audio_input_filterer = mmi_package.audio_input_filter_node:main'
        ],
    },
)
