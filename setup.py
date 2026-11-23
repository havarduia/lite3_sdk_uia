# Metadata here, not in pyproject.toml: the robot's Ubuntu 20.04 has setuptools 45 and
# pip 20, which cannot read [project] or install it editable without a setup.py.
# rclpy and the ROS message packages come from ROS 2 Foxy, not pip (docs/setup.md).
from setuptools import setup

setup(
    name='lite3-sdk-uia',
    version='0.1.0',
    description='Python SDK for the DEEP Robotics Lite3 Pro (ROS 2 Foxy)',
    python_requires='>=3.8',
    packages=['lite3'],
)
