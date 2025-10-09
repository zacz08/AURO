from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_path = get_package_share_directory('week_2')
    world_file = os.path.join(pkg_path, 'worlds', 'minimal_world_2.sdf')
    models_path = os.path.join(pkg_path, 'models')

    # 设置 Gazebo 模型路径
    set_gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{models_path}:/usr/share/gazebo-11/models'
    )

    # Fortress 正确启动方式 (不带引号)
    start_gz = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py',
            f'gz_args:=-r {world_file}'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_gz_model_path,
        start_gz
    ])