import asyncio
from typing import Dict, Any

import gpiozero  # type: ignore
from yaqd_core import UsesI2C, UsesSerial, HasLimits, HasPosition, IsHomeable, IsDaemon
from adafruit_motor import stepper  # type: ignore


styles = {
    "DOUBLE": stepper.DOUBLE,
    "SINGLE": stepper.SINGLE,
    "INTERLEAVE": stepper.INTERLEAVE,
    "MICROSTEP": stepper.MICROSTEP,
}


class AdafruitStepperMotorHat(UsesI2C, UsesSerial, IsHomeable, HasLimits, HasPosition, IsDaemon):
    _kind = "adafruit-stepper-motor-hat"

    def __init__(self, name, config, config_filepath):
        # adafruit_motorkit raises an exception if not rpi
        from adafruit_motorkit import MotorKit  # type: ignore

        super().__init__(name, config, config_filepath)
        self.microsteps = config["microsteps"]
        self._kit = MotorKit(
            address=config["i2c_addr"],
            steppers_microsteps=self.microsteps,
        )
        self._stepper = getattr(self._kit, f"stepper{config['stepper_index']}")
        if config["style"] in ["DOUBLE", "SINGLE"]:  # full steps only
            self.step_size = self.microsteps
        elif config["style"] == "INTERLEAVE":  # half stepping
            self.step_size = self.microsteps // 2
        elif config["style"] == "MICROSTEP":  # microstepping
            self.step_size = 1
        self.style = styles[config["style"]]
        self.steps_per_unit = config["steps_per_unit"]
        self._units = config["units"]
        self._lock = asyncio.Lock()
        self._lower_pin = gpiozero.InputDevice(config["lower_limit_switch"]["pin"], pull_up=True)
        if config.get("upper_limit_switch"):
            self._upper_pin = gpiozero.InputDevice(
                config["upper_limit_switch"]["pin"], pull_up=True
            )

    def _do_step(self, backward=False):
        steps = self.to_usteps(self._state["position"])
        direction = stepper.BACKWARD if backward else stepper.FORWARD
        if direction == stepper.BACKWARD and self._get_lower_limit_switch():
            return
        elif direction == stepper.FORWARD and self._get_upper_limit_switch():
            return
        self._stepper.onestep(direction=direction, style=self.style)
        steps += self.step_size if direction == stepper.FORWARD else -self.step_size
        self._state["position"] = self.to_units(steps)
        self.logger.debug(f"{self._state['position']}")

    def to_units(self, usteps):
        return usteps / self.steps_per_unit / self.microsteps

    def to_usteps(self, units):
        return round(units * self.steps_per_unit * self.microsteps)

    def _get_lower_limit_switch(self):
        # TODO: invert
        return self._lower_pin.value

    def _get_upper_limit_switch(self):
        if not self._config.get("upper_limit_switch"):
            return False
        # TODO: invert
        return self._upper_pin.value

    def home(self):
        self._busy = True
        self._loop.create_task(self._home())

    async def _home(self):
        prev_position = self._state["position"]
        async with self._lock:
            self._busy = True
            while True:
                await asyncio.sleep(0)
                self._do_step(backward=True)
                if self._state["position"] == prev_position:
                    break
                prev_position = self._state["position"]
            self._state["position"] = 0

    def _set_position(self, position):
        # destination gets set by parent
        # update state actually steps motor
        pass

    async def update_state(self):
        while True:
            async with self._lock:
                while (
                    abs(self.to_usteps(self._state["position"] - self._state["destination"]))
                    >= self.step_size
                ):
                    self._busy = True
                    await asyncio.sleep(0)
                    self._do_step(
                        self.to_usteps(self._state["position"])
                        > self.to_usteps(self._state["destination"])
                    )
                self._busy = False
            await self._busy_sig.wait()

    def direct_serial_write(self, msg):
        pass

    def close(self):
        self._stepper.release()
