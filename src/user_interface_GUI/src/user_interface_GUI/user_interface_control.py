#!/usr/bin/env python3

# Standard Libraries
import os
import rospy
import rospkg

# Qt Libraries
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

# Rospy msgs
from std_msgs.msg import String
from std_msgs.msg import Int16


class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('user_interface_GUI'), 'resources',
                               'graphical_user_interface.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.

        # ROS Publishers and Subscribers -------------->

        self.coordinator_to_gui_sub = rospy.Subscriber('/coordinator_to_gui', String, self.comm_to_gui_callback)
        self.gui_to_coordinator_pub = rospy.Publisher('gui_to_coordinator', String, queue_size=1000)
        self.send_num_wp_pub_cylinder = rospy.Publisher('/num_wp_cyl', Int16, queue_size=1000)
        self.send_num_wp_pub_rectangular = rospy.Publisher('/num_wp_rec', Int16, queue_size=1000)
        # --------------------------------------------->

        # GUI Buttons Connections ---------------------------------------------------->

        # UR5_Robot_1 Pick Workpiece Button
        self._widget.robot1_pick_wp_btn.clicked[bool].connect(self.robot1_pick_workpiece)
        # UR5_Robot_1 Assemble Workpiece Button
        self._widget.robot1_assemble_wp_btn.clicked[bool].connect(self.robot1_assemble_workpiece)

        # UR5_Robot_2 Pick Workpiece Button
        self._widget.robot2_pick_wp_btn.clicked[bool].connect(self.robot2_pick_workpiece)
        # UR5_Robot_2 Assemble Workpiece Button
        self._widget.robot2_assemble_wp_btn.clicked[bool].connect(self.robot2_assemble_workpiece)

        # Initialise Robots Buttons
        self._widget.initialise_robot_btn.clicked[bool].connect(self.initialise_robots)

        # Load Cylindrical Workpiece
        self._widget.load_cyl_wp_btn.clicked[bool].connect(self.load_cyl_workpiece)
        # Reset Cylindrical Workpiece
        self._widget.reset_cyl_wp_btn.clicked[bool].connect(self.reset_workpiece)

        # Load Rectangular Workpiece
        self._widget.load_rec_wp_btn.clicked[bool].connect(self.load_rec_workpiece)
        # Reset Rectangular Workpiece
        self._widget.reset_rec_wp_btn.clicked[bool].connect(self.reset_workpiece)

        # ---------------------------------------------------------------------------->

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    # Utility functions for User Interface

    def publish_msg_to_coordinator(self, msg):
        msg_to_publish = msg
        self.gui_to_coordinator_pub.publish(msg_to_publish)

    # Robot_ 1 --------------------------------->
    def robot1_pick_workpiece(self):
        self.publish_msg_to_coordinator('ROB1 - approach and pick workpiece')

    def robot1_assemble_workpiece(self):
        self.publish_msg_to_coordinator('ROB1 - assemble the part_ROB1')
    #  ------------------------------------------>

    # Robot_ 2 ---------------------------------->
    def robot2_pick_workpiece(self):
        self.publish_msg_to_coordinator('ROB2 - approach and pick workpiece')

    def robot2_assemble_workpiece(self):
        self.publish_msg_to_coordinator('ROB2 - assemble the part_ROB1')
    # ------------------------------------------->

    def initialise_robots(self):
        self.publish_msg_to_coordinator('Initialise the Robots')

    def load_cyl_workpiece(self):
        num_wp_type1 = self._widget.cyl_type1_nwp_txtbx.text()
        num_wp_type2 = self._widget.cyl_type2_nwp_txtbx.text()
        msg_type1 = int(num_wp_type1)
        msg_type2 = int(num_wp_type2)
        self.send_num_wp_pub_cylinder.publish(msg_type1)
        self.send_num_wp_pub_cylinder.publish(msg_type2)

    def load_rec_workpiece(self):
        num_wp_type1 = self._widget.rec_type1_nwp_txtbx.text()
        num_wp_type2 = self._widget.rec_type2_nwp_txtbx.text()
        msg_type1 = int(num_wp_type1)
        msg_type2 = int(num_wp_type2)
        self.send_num_wp_pub_rectangular.publish(msg_type1)
        self.send_num_wp_pub_rectangular.publish(msg_type2)

    def reset_workpiece(self):
        self._widget.cyl_type1_nwp_txtbx.clear()
        self._widget.cyl_type2_nwp_txtbx.clear()

        self._widget.rec_type1_nwp_txtbx.clear()
        self._widget.rec_type2_nwp_txtbx.clear()

    # Update Process Status TextBox ------------------------>

    def comm_to_gui_callback(self, msg):
        status = msg.data
        self._widget.process_status_txtbx.setText(status)
        pass

    def shutdown_plugin(self):
        # TODO unregister all publishers and subscribers here
        self.gui_to_coordinator_pub.unregister()
        self.coordinator_to_gui_sub.unregister()
        self.send_num_wp_pub_cylinder.unregister()
        self.send_num_wp_pub_rectangular.unregister()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # def trigger_configuration(self):
    # Comment in to signal that the plugin has a way to configure
    # This will enable a setting button (gear icon) in each dock widget title bar
    # Usually used to open a modal configuration dialog
