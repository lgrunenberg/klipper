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
    "PROTECT_RELEASE": {"addr": 0x3D, "len": 3},
    "PROTECT_STATE": {"addr": 0x3E, "len": 3},
}

WriteRegisterAddrs = {
    "CALIBRATE": 0x80,
    "SET_CURRENT": 0x83,
    "SET_MODE": 0x82,
    "SET_MSTEP": 0x84,
    "SET_EN": 0x85,
    "SET_DIR": 0x86,
    "SET_SDD": 0x87,
    "SET_PROTECT": 0x88,
    "SET_MPLYER": 0x89,
    "RESTORE": 0x3F,
    "SET_ADDR": 0x8B,
    "SET_ZEROMODE": 0x90,
    "SET_ZEROVAL": 0x91,
    "SET_ZEROSPEED": 0x92,
    "SET_ZERODIR": 0x93,
    "GOTO_ZERO": 0x94,
    "SET_KP": 0xA1,
    "SET_KI": 0xA2,
    "SET_KD": 0xA3,
    "SET_ACC": 0xA4,
    "SET_TORQ": 0xA5,
    "UART_EN": 0xF3,
    "UART_RUN": 0xF6,
    "UART_STOP": 0xF7,
    "UART_CLEAR": 0xFF,
    "UART_MOVE": 0xFD,
}

WriteRegisters = {
    key: {"addr": hex(value)} for key, value in WriteRegisterAddrs.items()
}

Registers = dict(ReadRegisters, **WriteRegisters)

MAX_CURRENT = 3.000

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

    def _handle_connect(self):
        try:
            self.set_current(self.current, None)
        except self.printer.command_error as e:
            logging.info("Servo42C %s failed to init: %s", self.name, str(e))

    def get_current(self):
        return self.current

    def set_current(self, run_current, print_time):
        val = math.ceil(run_current / 0.2)
        self.mcu_s42c.set_register("CURRENT", val, print_time)
        self.current = val * 0.2
        # Register commands
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
