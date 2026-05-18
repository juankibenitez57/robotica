from setuptools import find_packages, setup

package_name = 'image_folder_publisher'

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
    maintainer='juanks',
    maintainer_email='juankibenitez57@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'folder_image_publisher = image_folder_publisher.folder_image_publisher:main',
            'CameraSubscriber = image_folder_publisher.CameraSubscriber:main',
        ],
    },
)
