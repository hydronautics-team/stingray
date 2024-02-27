from rclpy.logging import get_logger
import time


class StateAction():
    def __init__(self,
                 type: str = "default",
                 **kwargs):
        """State class"""
        self.type = type
        if kwargs:
            get_logger("fsm").warning(
                f"{self.type} state action unused kwargs: {kwargs}")

    def __repr__(self) -> str:
        return f"type: {self.type}"

    def execute(self) -> bool:
        raise NotImplementedError(
            f"execute method not implemented for {self.type} state action")


class DurationAction(StateAction):
    def __init__(self,
                 type: str = "Duration",
                 duration: float = 60,
                 **kwargs):
        super().__init__(type=type, **kwargs)
        self.duration = duration

    def __repr__(self) -> str:
        return f"type: {self.type}, duration: {self.duration}"

    def execute(self) -> bool:
        get_logger("fsm").info(
            f"Executing {self.type} state action for {self.duration} seconds")
        time.sleep(self.duration)
        return True


class InitIMUAction(DurationAction):
    def __init__(self,
                 type: str = "InitIMU",
                 **kwargs):
        super().__init__(type=type, **kwargs)

    def __repr__(self) -> str:
        return f"type: {self.type}, duration: {self.duration}"

    def execute(self) -> bool:
        get_logger("fsm").info(f"Initializing IMU")
        return super().execute()


class EnableStabilizationAction(StateAction):
    def __init__(self,
                 type: str = "EnableStabilization",
                 surge: bool = False,
                 sway: bool = False,
                 depth: bool = False,
                 roll: bool = False,
                 pitch: bool = False,
                 yaw: bool = False,
                 **kwargs):
        super().__init__(type=type, **kwargs)

        self.surge = surge
        self.sway = sway
        self.depth = depth
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def __repr__(self) -> str:
        return f"type: {self.type}, surge: {self.surge}, sway: {self.sway}, depth: {self.depth}, roll: {self.roll}, pitch: {self.pitch}, yaw: {self.yaw}"

    def execute(self) -> bool:
        get_logger("fsm").info(f"Enabling stabilization")
        return True


class ThrusterIndicationAction(StateAction):
    def __init__(self,
                 type: str = "ThrusterIndication",
                 repeat: int = 1,
                 **kwargs):
        super().__init__(type=type, **kwargs)
        self.repeat = repeat

    def __repr__(self) -> str:
        return f"type: {self.type}, repeat: {self.repeat}"

    def execute(self) -> bool:
        get_logger("fsm").info(f"Thruster indication")
        return True
    

class MoveAction(StateAction):
    def __init__(self,
                 type: str = "Move",
                 linear: tuple[float,float,float] = (0,0,0),
                 angular: tuple[float,float,float] = (0,0,0),
                 **kwargs):
        super().__init__(type=type, **kwargs)
        self.linear = linear
        self.angular = angular

    def __repr__(self) -> str:
        return f"type: {self.type}, linear: {self.linear}, angular: {self.angular}"

    def execute(self) -> bool:
        get_logger("fsm").info(f"Move")
        return True


def create_action(action: dict) -> StateAction:
    if action['type'] == "Duration":
        return DurationAction(**action)
    elif action['type'] == "InitIMU":
        return InitIMUAction(**action)
    elif action['type'] == "EnableStabilization":
        return EnableStabilizationAction(**action)
    elif action['type'] == "ThrusterIndication":
        return ThrusterIndicationAction(**action)
    elif action['type'] == "Move":
        return MoveAction(**action)
    else:
        raise NotImplementedError(
            f"Action type {action['type']} not implemented")
    return None
