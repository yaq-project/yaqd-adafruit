import asyncio
from typing import Dict, Any

import yaqc  # type: ignore
from yaqd_core import UsesI2C, UsesSerial, HasLimits, HasPosition, IsHomeable, IsDaemon


class AdafruitStepperMotorHat(UsesI2C, UsesSerial, IsHomeable, HasLimits, HasPosition, IsDaemon):
    _kind = "adafruit-stepper-motor-hat"

    def __init__(self, name, config, config_filepath):
        # adafruit_motorkit raises an exception if not rpi
        from adafruit_motorkit import MotorKit  # type: ignore

        super().__init__(name, config, config_filepath)
        self._kit = MotorKit(address=config["i2c_addr"], steppers_microsteps=config["microsteps"])
        self._stepper = getattr(self._kit, f"stepper{config['stepper_index']}")
        self._homing = False
        self._lower_client = None
        self._upper_client = None

    async def _do_step(self, backward=False):
        from adafruit_motor import stepper  # type: ignore
        direction = stepper.BACKWARD if backward else stepper.FORWARD
        if direction == stepper.BACKWARD and await self._get_lower_limit_switch():
            return
        elif direction == stepper.FORWARD and await self._get_upper_limit_switch():
            return
        self._stepper.onestep(direction=direction, style=stepper.MICROSTEP)
        self._state["position"] += 1 if direction == stepper.FORWARD else -1


    async def _get_lower_limit_switch(self):
        if not self._lower_client:
            port = self._config["lower_limit_switch"]["port"]
            self._lower_client = yaqc.Client(port)
        self._lower_client.measure()
        while self._lower_client.busy():
            await asyncio.sleep(0)
        # TODO: invert
        return self._lower_client.get_measured()[self._config["lower_limit_switch"]["channel"]]

    async def _get_upper_limit_switch(self):
        if not self._config.get("upper_limit_switch"):
            return False
        if not self._upper_client:
            port = self._config["upper_limit_switch"]["port"]
            self._upper_client = yaqc.Client(port)
        self._upper_client.measure()
        while self._upper_client.busy():
            await asyncio.sleep(0)
        # TODO: invert
        return self._upper_client.get_measured()[self._config["upper_limit_switch"]["channel"]]

    def home(self):
        self._busy = True
        self._loop.create_task(self._home())

    async def _home(self):
        self._homing = True
        prev_position = self._state["position"]
        while True:
            await self._do_step(backward=True)
            if self._state["position"] == prev_position:
                break
            prev_position = self._state["position"]
        self._state["position"] = 0
        self._homing = False

    def _set_position(self, position):
        # destination gets set by parent
        # update state actually steps motor
        pass

    async def update_state(self):
        while True:
            while self._state["position"] != self._state["destination"]:
                if self._homing:
                    await asyncio.sleep(1)
                    continue
                self._busy = True
                await self._do_step(self._state["position"] > self._state["destination"])
            await asyncio.sleep(0.01)
            #await self._busy_sig.wait()

    def direct_serial_write(self, msg):
        pass
