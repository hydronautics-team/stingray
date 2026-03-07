
class State:
    ALL = "*"
    IDLE = "IDLE"
    OK = "OK"
    FAILED = "FAILED"

    @staticmethod
    def aslist():
        return [State.ALL, State.IDLE, State.OK, State.FAILED]


class Transition:
    ok = "ok"
    fail = "fail"
    reset = "reset"
    timeout = "timeout"
