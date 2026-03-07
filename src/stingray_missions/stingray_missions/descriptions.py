from pathlib import Path
from rclpy.logging import get_logger

from stingray_missions.fsm_states import State, Transition
from stingray_utils.config import load_yaml


class StateDescription():
    name: str
    timeout: float
    action_args: dict

    def __init__(self,
                 name: str = "",
                 timeout: float = None,
                 action: dict = None,
                 **kwargs):
        """State class"""
        self.name = name.upper()
        self.timeout = timeout
        self.action_args = action if action else None

        if kwargs:
            get_logger("fsm").warning(
                f"{self.name} state unused kwargs: {kwargs}")

    def __repr__(self) -> str:
        return f"""
                        State: {self.name}
                            timeout: {self.timeout}
                            action:
                                {self.action_args}
        """


class MissionDescription:
    loaded_yamls = {}

    def __init__(self,
                 name: str = "",
                 initial: str = "",
                 states: dict[str, dict] = None,
                 transitions: list[dict[str, str]] = None,
                 **kwargs
                 ):
        """
        Mission class for executing a mission from a config file
        """
        if states is None:
            states = {}
        if transitions is None:
            transitions = []

        self.name = name.upper()
        self.initial_state = self._custom_state_name(initial)

        # Список состояний
        self.states = [
            StateDescription(
                name=self._custom_state_name(s_name),
                **s_params
            )
            for s_name, s_params in states.items()
        ]

        # Список переходов
        self.mission_transitions = []
        for transition in transitions:
            trigger = transition['trigger']

            # Источник может быть списком или строкой
            if isinstance(transition['source'], list):
                source = [self._custom_state_name(state)
                          for state in transition['source']]
            else:
                if transition['source'] == State.ALL:
                    source = [state.name for state in self.states]
                else:
                    source = self._custom_state_name(transition['source'])

            # Целевое состояние
            if transition['dest'] in State.aslist():
                dest = transition['dest']
            else:
                dest = self._custom_state_name(transition['dest'])

            self.mission_transitions.append({
                'trigger': trigger,
                'source': source,
                'dest': dest
            })

        # После чтения конфигурации добавляем fail-переходы для всех состояний
        self.add_fail_transitions()

        if kwargs:
            get_logger("fsm").warning(
                f"{self.name} mission unused kwargs: {kwargs}")

    def _custom_state_name(self, name: str) -> str:
        """
        Формируем имя состояния с префиксом миссии,
        кроме случаев, когда имя совпадает с глобальными 'OK' / 'FAILED',
        если хотим избежать 'OK|OK' и т.п.
        """
        # Если это итоговые имена, оставляем их без префикса
        if name.upper() in [State.OK, State.FAILED]:
            return name.upper()

        return f"{self.name}|{name}".upper()

    def add_fail_transitions(self):
        """
        Убедиться, что у каждого состояния есть триггер 'fail', ведущий в глобальный FAILED.
        """
        # Пробегаемся по всем состояниям
        for state in self.states:
            # Проверяем, есть ли уже переход c trigger='fail' для этого состояния
            exists_fail = any(
                t for t in self.mission_transitions
                if t['trigger'] == Transition.fail and (
                    (isinstance(t['source'], list) and state.name in t['source']) or
                    (t['source'] == state.name)
                )
            )
            # Если нет — добавляем
            if not exists_fail:
                self.mission_transitions.append({
                    'trigger': Transition.fail,
                    'source': state.name,
                    'dest': State.FAILED  # Глобальное имя FAILED
                })

    @staticmethod
    def load(config_name: str,
             package_name: str = "stingray_missions",
             custom_name: str = None
             ) -> "MissionDescription":
        if custom_name is None:
            custom_name = Path(config_name).stem
        # Считываем YAML один раз
        if config_name not in MissionDescription.loaded_yamls:
            MissionDescription.loaded_yamls[config_name] = load_yaml(
                config_path=f"configs/missions/{config_name}",
                package_name=package_name
            )

        return MissionDescription(
            name=custom_name,
            **MissionDescription.loaded_yamls[config_name]
        )

    @property
    def transitions(self) -> list[list[str, str, str]]:
        """
        Возвращает список переходов в формате [trigger, source, dest].
        """
        return [
            [t['trigger'], t['source'], t['dest']]
            for t in self.mission_transitions
        ]

    def set_OK_outcome(self, state: str):
        """
        Заменить все dest=OK в конфигурации на заданное состояние.
        """
        for transition in self.mission_transitions:
            if transition['dest'] == State.OK:
                transition['dest'] = state

    def set_FAILED_outcome(self, state: str):
        """
        Заменить все dest=FAILED в конфигурации на заданное состояние.
        """
        for transition in self.mission_transitions:
            if transition['dest'] == State.FAILED:
                transition['dest'] = state


class ScenarioDescription:
    loaded_yamls = {}
    loaded_oks = {}
    loaded_faileds = {}

    def __init__(self,
                 name: str = "",
                 initial: str = "",
                 ok_state_package_name: str = "stingray_missions",
                 failed_state_package_name: str = "stingray_missions",
                 missions: dict[str, dict] = None,
                 transitions: list[dict] = None,
                 **kwargs
                 ):
        """
        Scenario class for executing a scenario from a config file
        """
        if missions is None:
            missions = {}
        if transitions is None:
            transitions = []

        self.name = name.upper()
        self.scenario_transitions = transitions
        self.ok_state_package_name = ok_state_package_name
        self.failed_state_package_name = failed_state_package_name

        # Загружаем миссии OK и FAILED только один раз на пакет
        if ok_state_package_name not in self.loaded_oks:
            self.loaded_oks[ok_state_package_name] = MissionDescription.load(
                config_name="ok",
                package_name=ok_state_package_name,
                custom_name="OK"
            )
        if failed_state_package_name not in self.loaded_faileds:
            self.loaded_faileds[failed_state_package_name] = MissionDescription.load(
                config_name="failed",
                package_name=failed_state_package_name,
                custom_name="FAILED"
            )

        # Загружаем миссии сценария
        self.missions = {}
        for m_name, m_params in missions.items():
            # Префиксируем имя миссии именем сценария, кроме случаев OK/FAILED
            custom_name = self._custom_mission_name(m_name)
            self.missions[custom_name] = MissionDescription.load(
                config_name=m_params.get("config_name", ""),
                package_name=m_params.get("package_name", "stingray_missions"),
                custom_name=custom_name
            )

        # Пробуем автоматически добавить переход на FAILED из любой миссии,
        # если outcome=FAILED
        self.scenario_transitions.append({
            'source': list(missions.keys()),
            'outcome': State.FAILED,
            'dest': State.FAILED
        })

        # Начальная миссия (тоже с префиксом сценария)
        self.initial_mission = self._custom_mission_name(initial)

        if kwargs:
            get_logger("fsm").warning(
                f"{self.name} scenario unused kwargs: {kwargs}")

    def _custom_mission_name(self, name: str) -> str:
        """
        Формируем имя миссии с префиксом сценария.
        Если миссия называется OK или FAILED, можно оставить без префикса,
        чтобы избежать OK|OK и т.п.
        """
        if name.upper() in [State.OK, State.FAILED]:
            return name.upper()
        return f"{self.name}|{name}".upper()

    def __repr__(self) -> str:
        missions_print = "\n".join(
            [f"{mission}" for mission in self.missions.values()]
        )
        return f"""
        Scenario: {self.name}
            {missions_print}
        """

    @staticmethod
    def load(config_name: str,
             package_name: str = "stingray_missions",
             custom_name: str = None
             ) -> "ScenarioDescription":
        if custom_name is None:
            custom_name = Path(config_name).stem

        # Считываем конфиг один раз
        if config_name not in ScenarioDescription.loaded_yamls:
            ScenarioDescription.loaded_yamls[config_name] = load_yaml(
                config_path=f"configs/scenarios/{config_name}",
                package_name=package_name
            )

        return ScenarioDescription(
            name=custom_name,
            **ScenarioDescription.loaded_yamls[config_name]
        )

    @property
    def initial_state(self) -> str:
        """
        Начальное состояние — это начальное состояние стартовой миссии.
        """
        return self.missions[self.initial_mission].initial_state

    @property
    def states(self) -> list[StateDescription]:
        """Возвращает все состояния всех миссий сценария."""
        all_states = []
        for mission in self.missions.values():
            all_states.extend(mission.states)
        return all_states

    @property
    def transitions(self) -> list[list[str, str, str]]:
        """
        Формирует итоговый список переходов FSM:
        1. Переход из IDLE в initial_state
        2. Обработка transitions из сценария (маппинг outcome -> dest)
        3. Все переходы внутренних миссий
        """
        scenario_transitions = [
            [self.name.lower(), State.IDLE, self.initial_state]
        ]

        for transition in self.scenario_transitions:
            source = transition['source']
            outcome = transition['outcome']
            dest = transition['dest']

            # source может быть строкой или списком
            if not isinstance(source, list):
                source = [source]

            match dest:
                case State.IDLE:
                    # outcome=OK => отправляем миссию в IDLE
                    for mission_name in source:
                        mission_full_name = self._custom_mission_name(mission_name)
                        self.missions[mission_full_name].set_OK_outcome(State.IDLE)

                case _:
                    # outcome = OK или FAILED (или что-то ещё) => разные ветки
                    match outcome.upper():
                        case State.OK:
                            # Выход из миссии по ok
                            if dest == State.OK:
                                # Тогда цепляем к миссии OK (загруженной из пакета)
                                target_init = self.loaded_oks[
                                    self.ok_state_package_name
                                ].initial_state
                            elif dest == State.FAILED:
                                # Редкий случай: outcome=OK, но dest=FAILED
                                # Посылаем в FAILED-миссию
                                target_init = self.loaded_faileds[
                                    self.failed_state_package_name
                                ].initial_state
                            else:
                                # Переход в другую миссию сценария
                                target_init = self.missions[
                                    self._custom_mission_name(dest)
                                ].initial_state

                            for mission_name in source:
                                mission_full_name = self._custom_mission_name(mission_name)
                                self.missions[mission_full_name].set_OK_outcome(target_init)

                        case State.FAILED:
                            # Выход из миссии по failed
                            if dest == State.OK:
                                # Редкий случай: outcome=FAILED, но dest=OK
                                target_init = self.loaded_oks[
                                    self.ok_state_package_name
                                ].initial_state
                            elif dest == State.FAILED:
                                # outcome=FAILED, dest=FAILED
                                # Переходим на миссию FAILED
                                target_init = self.loaded_faileds[
                                    self.failed_state_package_name
                                ].initial_state
                            else:
                                # Переход в другую миссию
                                target_init = self.missions[
                                    self._custom_mission_name(dest)
                                ].initial_state

                            for mission_name in source:
                                mission_full_name = self._custom_mission_name(mission_name)
                                self.missions[mission_full_name].set_FAILED_outcome(target_init)

                        case _:
                            raise ValueError(
                                f"Invalid outcome: {outcome} in scenario {self.name}"
                            )

        # Добавляем все переходы из миссий
        for mission in self.missions.values():
            scenario_transitions.extend(mission.transitions)

        return scenario_transitions
