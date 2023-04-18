import asyncio
from typing import Dict, Any

import gpiozero  # type: ignore
from yaqd_core import UsesI2C, UsesSerial, HasLimits, HasPosition, IsHomeable, IsDaemon
from adafruit_motor import stepper  # type: ignore


class AdafruitStepperMotorHat(UsesI2C, UsesSerial, IsHomeable, HasLimits, HasPosition, IsDaemon):
    _kind = "adafruit-stepper-motor-hat"

    def __init__(self, name, config, config_filepath):
        # adafruit_motorkit raises an exception if not rpi
        from adafruit_motorkit import MotorKit  # type: ignore

        super().__init__(name, config, config_filepath)
        try:
            self.microsteps = config["microsteps"]
            self.style = config["style"]
            if self.style in ["DOUBLE", "SINGLE"]:  # full steps only
                self.step_size = self.microsteps
            elif self.style == "INTERLEAVE":  # half stepping
                self.step_size = self.microsteps // 2
            elif self.style == "MICROSTEPS":  # microstepping
                self.step_size = 1
            self._kit = MotorKit(
                address=config["i2c_addr"],
                steppers_microsteps=self.microsteps,
            )
            self.steps_per_unit = config["steps_per_unit"]
            self._units = config["units"]
            self._lock = asyncio.Lock()
            self._lower_pin = gpiozero.InputDevice(
                config["lower_limit_switch"]["pin"], pull_up=True
            )
            if config.get("upper_limit_switch"):
                self._upper_pin = gpiozero.InputDevice(
                    config["upper_limit_switch"]["pin"], pull_up=True
                )
        except Exception as e:
            self.logger.error(e)

    async def _do_step(self, backward=False):
        steps = self.to_usteps(self._state["position"])
        direction = stepper.BACKWARD if backward else stepper.FORWARD
        if direction == stepper.BACKWARD and await self._get_lower_limit_switch():
            return
        elif direction == stepper.FORWARD and await self._get_upper_limit_switch():
            return
        try:
            self._stepper.onestep(direction=direction, style=self.style)
        except Exception as e:
            self.logger.error(e)
        steps += self.step_size if direction == stepper.FORWARD else -self.step_size
        self._state["position"] = self.to_units(steps)
        self.logger.debug(f"{self._state['position']}")

    def to_units(self, usteps):
        return usteps / self.steps_per_unit / self.microsteps

    def to_usteps(self, units):
        return round(units * self.steps_per_unit * self.microsteps)

    def release(self):
        """for debugging purposes only"""
        self._stepper.release()

    async def _get_lower_limit_switch(self):
        # TODO: invert
        return self._lower_pin.value

    async def _get_upper_limit_switch(self):
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
                await self._do_step(backward=True)
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
                while self.to_usteps(self._state["position"]) != self.to_usteps(
                    self._state["destination"]
                ):
                    self._busy = True
                    await asyncio.sleep(0)
                    await self._do_step(
                        self.to_usteps(self._state["position"])
                        > self.to_usteps(self._state["destination"])
                    )

                self._busy = False
            await self._busy_sig.wait()

    def direct_serial_write(self, msg):
        pass

    def close(self):
        self._stepper.release()
