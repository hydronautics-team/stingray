from os import path
from json import load
from rospkg import RosPack


def load_config(config_name='ros.json', package_name="stingray_resources"):
    stingray_resources_path = RosPack().get_path(package_name)
    with open(path.join(stingray_resources_path, "configs", config_name)) as f:
        config = load(f)
    return config
