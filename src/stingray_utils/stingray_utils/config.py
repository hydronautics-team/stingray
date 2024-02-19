from pathlib import Path
from rclpy.logging import get_logger
from rospkg import RosPack
from ament_index_python.packages import get_package_share_directory
import yaml
try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader

# TODO move to stingray_utils


def load_yaml(config_path: str, package_name: str) -> dict:
    pakage_path = get_package_share_directory(package_name)
    config_path = Path(config_path).with_suffix('.yaml')
    get_logger('stingray_utils').info(f'load config from {pakage_path}/{config_path}')
    with open(Path(pakage_path, config_path), 'r') as f:
        return yaml.load(f, Loader=Loader)


class AbstractConfig():
    def __init__(self,
                 name: str
                 ):
        self._name = name
        self._config_items = load_yaml(config_name=name)
        for k, v in self._config_items.items():
            setattr(self, k, v)

    def __repr__(self) -> str:
        return f'{self._name}: {self._config_items}'


class StingrayConfig():
    robot: AbstractConfig
    ros: AbstractConfig

    def __init__(self):
        StingrayConfig.robot = AbstractConfig(name='robot.yaml')
        StingrayConfig.ros = AbstractConfig(name='ros.yaml')

    def __repr__(self) -> str:
        return f'robot: {StingrayConfig.robot}, ros: {StingrayConfig.ros}'


# StingrayConfig()
