#!/usr/bin/python3
# -*- coding:utf-8 -*-

"""
 Copyright 2025 Author lio

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      https://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 """

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='asr',
            executable='asr',
            parameters=[{'service_provider': 'ifly', # baidu ifly
                         'baidu_api_key': 'yqapHe9IKnKZZdKIkvv0r7YQ',
                         'baidu_secret_key': 'uWXoblwoT1YR5BZPoN3SUrttXfPxDm3I',
                         'ifly_appid': '6f20e2c2',
                         'ifly_apikey': "fc5fc33f5d17487a93263120e7c6d393",
                         'ifly_apisecret': "NzMwZTYwYTY0MWZkNjYyMGM0YjJjMjE4"}],
            output='screen')
    ])
