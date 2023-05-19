#!/usr/bin/env python3
# Copyright (C) 2016 Toyota Motor Corporation

try:
    from traitlets.config.loader import Config
except ImportError:
    from IPython.config.loader import Config
try:
    get_ipython
except NameError:
    nested = 0
    cfg = Config()
else:
    cfg = Config()
    nested = 1


from hsrb_interface_py import Robot
from hsrb_interface_py import robot as _robot

from IPython.terminal.embed import InteractiveShellEmbed

import rclpy

shell = InteractiveShellEmbed(config=cfg,
                              banner1="HSR-B Interactive Shell 0.2.0",
                              exit_msg="Leaving HSR-B Interactive Shell")

LOGO = r"""
      ____________  ______  _________       __  _______ ____
     /_  __/ __ \ \/ / __ \/_  __/   |     / / / / ___// __ \
      / / / / / /\  / / / / / / / /| |    / /_/ /\__ \/ /_/ /
     / / / /_/ / / / /_/ / / / / ___ |   / __  /___/ / _, _/
    /_/  \____/ /_/\____/ /_/ /_/  |_|  /_/ /_//____/_/ |_|
"""


def main():
    _robot.enable_interactive()

    rclpy.init()
    with Robot() as robot:
        whole_body = robot.try_get('whole_body')  # noqa
        # omni_base = robot.try_get('omni_base')
        # collision_world = robot.try_get('global_collision_world')
        # suction = robot.try_get('suction')
        gripper = robot.try_get('gripper')  # noqa
        # wrist_wrench = robot.try_get('wrist_wrench')
        # marker = robot.try_get('marker')
        # battery = robot.try_get('battery')
        # tts = robot.try_get('default_tts')
        shell(LOGO)
