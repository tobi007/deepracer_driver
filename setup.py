from setuptools import setup

try:
    from setuptools import setup, find_packages
except ImportError:
    from distutils.core import setup, find_packages

REQUIRES_PYTHON = '>=3.5.0'
package_name = 'deepracer_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    python_requires=REQUIRES_PYTHON,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tobi-k',
    maintainer_email='tobi-k@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver=nodes.driver:main',
            'hsv_finder=nodes.hsv_finder:main'
        ],
    },
)
