from setuptools import find_packages, setup

package_name = 'cv_detection'

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
    maintainer='ndrusr',
    maintainer_email='ngweizheng@gmail.com',
    description='OpenCV module for person tracking',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_cv = cv_detection.hello_cv:main'
        ],
    },
)
