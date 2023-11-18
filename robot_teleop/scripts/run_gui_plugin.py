#!/usr/bin/env python3
import sys

from robot_teleop.rqt_plugin import GUIControllerPlugin
from rqt_gui.main import Main

"""
Starts rqt plugin.

run with command:
$ rqt --standalone robot_teleop
"""

main = Main(filename='robot_teleop')
sys.exit(main.main(sys.argv, standalone='robot_teleop'))