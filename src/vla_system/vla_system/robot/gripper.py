"""Small OnRobot RG2/RG6 Modbus adapter with explicit error checking."""

import time


class GripperError(RuntimeError):
    pass


class OnRobotRG:
    UNIT_ID = 65

    def __init__(self, model: str, ip: str, port: int = 502, timeout_s: float = 1.0):
        from pymodbus.client.sync import ModbusTcpClient

        model = model.lower()
        if model not in {"rg2", "rg6"}:
            raise ValueError("model must be rg2 or rg6")
        self.max_width_tenth_mm = 1100 if model == "rg2" else 1600
        self.max_force_tenth_n = 400 if model == "rg2" else 1200
        self.client = ModbusTcpClient(ip, port=port, timeout=timeout_s)
        if not self.client.connect():
            raise GripperError(f"cannot connect to gripper at {ip}:{port}")

    @staticmethod
    def _check(result, operation: str) -> None:
        if result is None or getattr(result, "isError", lambda: True)():
            raise GripperError(f"Modbus operation failed: {operation}")

    def move(self, width_mm: float, force_n: float) -> None:
        width = int(round(width_mm * 10.0))
        force = int(round(force_n * 10.0))
        if not 0 <= width <= self.max_width_tenth_mm:
            raise ValueError("gripper width out of range")
        if not 0 <= force <= self.max_force_tenth_n:
            raise ValueError("gripper force out of range")
        result = self.client.write_registers(
            address=0,
            values=[force, width, 16],
            unit=self.UNIT_ID,
        )
        self._check(result, "move")

    def open(self, force_n: float = 40.0) -> None:
        self.move(self.max_width_tenth_mm / 10.0, force_n)

    def close(self, force_n: float = 40.0) -> None:
        self.move(0.0, force_n)

    def wait_until_idle(self, timeout_s: float = 3.0, poll_s: float = 0.05) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            result = self.client.read_holding_registers(
                address=268,
                count=1,
                unit=self.UNIT_ID,
            )
            self._check(result, "read status")
            busy = bool(result.registers[0] & 0x0001)
            if not busy:
                return
            time.sleep(poll_s)
        raise GripperError("gripper motion timeout")

    def close_connection(self) -> None:
        self.client.close()
