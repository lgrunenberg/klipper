# SERVO42C configuration
#
# Copyright (C) 2023  Lucas Grunenberg <lucas.grunenberg@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import servo42c_uart

import logging
import math


ReadRegisters = {
    "ENC_VALUE": {"addr": 0x30, "len": 8},
    "PULSE_VALUE": {"addr": 0x33, "len": 6},
    "ERROR_VALUE": {"addr": 0x39, "len": 4},
    "EN_STATUS": {"addr": 0x3A, "len": 3},
    "LOCK_STATE": {"addr": 0x3D, "len": 3},
    "PROTECT_STATE": {"addr": 0x3E, "len": 3},
}

WriteRegisters = {
    "CALIBRATE": {"addr": 0x80, "len": 4},
    "SET_CURRENT": {"addr": 0x83, "len": 4},
    "SET_MODE": {"addr": 0x82, "len": 4},
    "SET_MSTEP": {"addr": 0x84, "len": 4},
    "SET_EN": {"addr": 0x85, "len": 4},
    "SET_DIR": {"addr": 0x86, "len": 4},
    "SET_SDD": {"addr": 0x87, "len": 4},
    "SET_LOCK": {"addr": 0x88, "len": 4},
    "SET_MPLYER": {"addr": 0x89, "len": 4},
    "RESTORE": {"addr": 0x3F, "len": 3},
    "SET_ADDR": {"addr": 0x8B, "len": 4},
    "SET_ZEROMODE": {"addr": 0x90, "len": 4},
    "SET_ZEROVAL": {"addr": 0x91, "len": 4},
    "SET_ZEROSPEED": {"addr": 0x92, "len": 4},
    "SET_ZERODIR": {"addr": 0x93, "len": 4},
    "GOTO_ZERO": {"addr": 0x94, "len": 4},
    "SET_KP": {"addr": 0xA1, "len": 5},
    "SET_KI": {"addr": 0xA2, "len": 5},
    "SET_KD": {"addr": 0xA3, "len": 5},
    "SET_ACC": {"addr": 0xA4, "len": 5},
    "SET_TORQ": {"addr": 0xA5, "len": 5},
    "UART_EN": {"addr": 0xF3, "len": 4},
    "UART_RUN": {"addr": 0xF6, "len": 4},
    "UART_STOP": {"addr": 0xF7, "len": 3},
    "UART_CLEAR": {"addr": 0xFF, "len": 4},
    "UART_MOVE": {"addr": 0xFD, "len": 8},
}


Registers = dict(ReadRegisters, **WriteRegisters)

MAX_CURRENT = 3.000
MAX_TORQUE = 1200

######################################################################
# SERVO42C printer object
######################################################################


class SERVO42C:
    def __init__(self, config):
        # Setup mcu communication
        self.mcu_s42c = servo42c_uart.MCU_S42C_uart(config, Registers, 255)
        self.printer = config.get_printer()

        self.stepper_name = " ".join(config.get_name().split()[1:])
        self.stepper_enable = self.printer.load_object(config, "stepper_enable")
        self.current = config.getfloat("current", above=0.0, maxval=MAX_CURRENT)
        self.torque = config.getfloat("torque", above=0.0, maxval=MAX_TORQUE)

        gcode = self.printer.lookup_object("gcode")
        self.name = config.get_name().split()[-1]
        gcode.register_mux_command(
            "SET_SERVO_CURRENT",
            "STEPPER",
            self.name,
            self.cmd_SET_SERVO_CURRENT,
            desc=self.cmd_SET_SERVO_CURRENT_help,
        )
        self.printer.register_event_handler("klippy:connect", self._handle_connect)

    def _do_enable(self, print_time):
        self.set_enable(True, print_time)

    def _do_disable(self, print_time):
        self.set_enable(False, print_time)

    def _handle_stepper_enable(self, print_time, is_enable):
        if is_enable:
            cb = lambda ev: self._do_enable(print_time)
        else:
            cb = lambda ev: self._do_disable(print_time)
        self.printer.get_reactor().register_callback(cb)

    def _handle_connect(self):
        # Check for soft stepper enable/disable
        enable_line = self.stepper_enable.lookup_enable(self.stepper_name)
        if not enable_line.has_dedicated_enable():
            enable_line.register_state_callback(self._handle_stepper_enable)
            logging.info("Enabling virtual enable for '%s'", self.stepper_name)
        try:
            self.set_current(self.current, None)
            self.set_torque(self.torque, None)
        except self.printer.command_error as e:
            logging.info("Servo42C %s failed to init: %s", self.name, str(e))

    def get_current(self):
        return self.current

    def set_current(self, run_current, print_time):
        val = math.ceil(run_current / 0.2)
        self.mcu_s42c.set_register("SET_CURRENT", val, print_time)
        self.current = val * 0.2
        # Register commands

    def get_motor_protect(self):
        return self.mcu_s42c.get_register("PROTECT_STATE")

    def get_motor_lock(self):
        return self.mcu_s42c.get_register("LOCK_STATE")

    def set_motor_lock(self, state, print_time):
        val = state
        self.mcu_s42c.set_register("SET_LOCK", val, print_time)

    def set_enable(self, state, print_time):
        val = 0
        if state:
            if self.get_motor_lock():
                self.set_motor_lock(False, print_time)
            val = 1
        try:
            self.mcu_s42c.set_register("SET_EN", val, print_time)
        except self.printer.command_error as e:
            logging.info("Servo42C %s failed to enable: %s", self.name, str(e))

    def set_torque(self, torque, print_time):
        val = round(torque)
        self.mcu_s42c.set_register("SET_TORQ", val, print_time)

    cmd_SET_SERVO_CURRENT_help = "Set the current of a Servo42C driver"

    def cmd_SET_SERVO_CURRENT(self, gcmd):
        prev_cur = self.get_current()
        run_current = gcmd.get_float("CURRENT", None, minval=0.0, maxval=MAX_CURRENT)

        if run_current is not None:
            toolhead = self.printer.lookup_object("toolhead")
            print_time = toolhead.get_last_move_time()
            self.set_current(run_current, print_time)
            prev_cur = self.get_current()
            # Report values
            gcmd.respond_info("Run Current: %0.2fA" % (prev_cur,))


def load_config_prefix(config):
    return SERVO42C(config)
