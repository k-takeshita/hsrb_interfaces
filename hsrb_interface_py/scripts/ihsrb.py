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



from IPython.terminal.embed import InteractiveShellEmbed
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

import math
import rclpy
from math import radians, degrees
from hsrb_interface import Robot, ItemTypes
from hsrb_interface import geometry

from hsrb_interface import robot as _robot

_robot.enable_interactive()

# メイン
def main(args=None):
    rclpy.init(args=args)
    with Robot() as robot:
        whole_body = robot.try_get('whole_body')
        omni_base = robot.try_get('omni_base')
        gripper = robot.try_get('gripper')
        shell(LOGO)
    
if __name__ == '__main__':
    main()
    
