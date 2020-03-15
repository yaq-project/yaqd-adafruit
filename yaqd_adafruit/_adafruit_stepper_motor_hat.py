__all__ = ["StepperMotorHat"]

import asyncio
from typing import Dict, Any

from adafruit_motorkit import MotorKit  # type: ignore
from adafruit_motor import stepper  # type: ignore
import yaqc  # type: ignore
from yaqd_core import ContinuousHardware, logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class AdafruitStepperMotorHat(ContinuousHardware):
    _kind = "adafruit_stepper-motor-hat"
    defaults: Dict[str, Any] = {
        "stepper_index": 1,
        "i2c_addr": 0x60,
        "microsteps": 16,
        "lower_limit_port": None,
        "upper_limit_port": None,
    }

    def __init__(self, name, config, config_filepath):
        super().__init__(name, config, config_filepath)
        self.kit = MotorKit(address=config["i2c_addr"], steppers_microsteps=config["microsteps"])
        self.stepper = getattr(self.kit, f"stepper{config['stepper_index']}")
        if config["lower_limit_port"]:
            self.lower_limit_switch = yaqc.Client(config["lower_limit_port"])
        if config["upper_limit_port"]:
            self.upper_limit_switch = yaqc.Client(config["upper_limit_port"])
        # TODO: maybe this should be a lock instead
        self._homing = False

    def _set_position(self, position):
        pass

    async def update_state(self):
        """Continually monitor and update the current daemon state."""
        # If there is no state to monitor continuously, delete this function
        while True:
            # Perform any updates to internal state
            while self.position != self.destination:
                if self._homing:
                    await asyncio.sleep(0.01)
                    continue
                self._busy = True
                self._do_step(self.position < self.destination)
                await asyncio.sleep(0)
            await self._busy_sig.wait()

    def _do_step(self, backward=False):
        direction = stepper.BACKWARD if backward else stepper.FORWARD
        if direction == stepper.BACKWARD and self.get_lower_limit_switch():
            return
        elif direction == stepper.FORWARD and self.get_upper_limit_switch():
            return
        self.stepper.onestep(direction=direction, style=stepper.MICROSTEP)

    def home(self):
        self._busy = True
        self._loop.create_task(self._home())

    async def _home(self):
        self._homing = True
        while not self.get_lower_limit_switch():
            self.do_step(backward=True)
            await asyncio.sleep(0)
        self.positon = 0
        self._homing = False

    def get_lower_limit_switch(self):
        return self.lower_limit_switch.get_value()

    def get_upper_limit_switch(self):
        return self.upper_limit_switch.get_value()
