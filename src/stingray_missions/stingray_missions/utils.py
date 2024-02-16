from pathlib import Path
from rclpy.logging import get_logger
from rospkg import RosPack
import yaml
try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader


def load_mission_config(name: str = "init.yaml", package_name="stingray_missions"):
    stingray_missions_path = RosPack().get_path(package_name)
    with open(Path(stingray_missions_path, name), 'r') as f:
        return yaml.load(f, Loader=Loader)


class AbstractConfig():
    def __init__(self,
                 name: str
                 ):
        self._name = name
        self._config_items = load_mission_config(name=name)
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
        return f'robot: {StingrayConfig.robot}, ros: {StingrayConfig.ros}, '


StingrayConfig()
