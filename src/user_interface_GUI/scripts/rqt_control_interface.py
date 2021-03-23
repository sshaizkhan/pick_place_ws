#!/usr/bin/env python3

import sys
from user_interface_GUI.src.user_interface_GUI.user_interface_control import MyPlugin
from rqt_gui.main import Main


plugin = 'user_interface_GUI'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))