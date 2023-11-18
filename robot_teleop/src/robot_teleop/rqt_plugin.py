#!/usr/bin/python3

import rospy
import os
import rospkg
from std_msgs.msg import String
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMessageBox

from std_msgs.msg import Float64

class GUIControllerPlugin(Plugin):
    """
    rqt plugin for GUI manual control over robot 
    movement. Displays up, down, left and right buttons 
    for direction control and a slider for velocity
    control. 
    """

    def __init__(self, context):
        super(GUIControllerPlugin, self).__init__(context)

        self.setObjectName('GUIControllerPlugin')

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

        # Loads the gui window created in QT 4 Designer 
        ui_file = os.path.join(rospkg.RosPack().get_path('robot_teleop'), 'resource', 'MedApp.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('GUIControllerPluginUI')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        # Wheel joint velocity publishers##  Publish coordinates
        self.publisher_dict = {} 
        # connection initialization
        self.init_signals()
        # publisher initialization
        self.init_ros()
        # set default velocity
        # Specific to my robot. Find out your desired velocity.
        #self.velocity = 0    # just to initialize the variable
        #self.upper_velocity_limit = 50.0 
        #self.update_velocity(10.0)


    def update_velocity(self, vel):
        """
        Sets private velocity, updates slider and
        velocity text field. 
        """
        # the textField will show values from 0 to 100
        # but the real velocity will be 0 to upper_velocity_limit
        #self.velocity = vel*( (self.upper_velocity_limit) * 0.01 )
        #self._widget.velocityValue.setText( str(vel) )
        #self._widget.velocitySlider.setValue(vel)

    def init_signals(self):
        """
        Method connects all nescessary signals from GUI.
        """
        # "pushButton_up" is the name of the button defined in the .ui file
        self._widget.pushButton_S1.pressed.connect(self.uno_pushed)
        self._widget.pushButton_S2.pressed.connect(self.due_pushed)
        self._widget.pushButton_R.pressed.connect(self.rec_pushed)
        self._widget.pushButton_C.pressed.connect(self.charge_pushed)
        
                            

    def init_ros(self):
        """
        Method initialises ROS interaction.
        """
        try:
            # my topics for commanding the velocity
            # insert ('/gazebo/default/pioneer2dx/vel_cmd') for your case
            self.publisher_dict['xyz'] = rospy.Publisher('/robot/command', String, queue_size=1)

        except rospy.ROSException as e:
            QMessageBox.about(self._widget, "ROS error", e.message)

    def keyPressEvent(self, event):
         if type(event) == QtGui.QKeyEvent:
             #here accept the event and do something
             print(event.key())
             event.accept()
         else:
             event.ignore()

    # The funtions called when button pressed
    def uno_pushed(self, checked = False):

        self.publisher_dict['xyz'].publish('Stanza 1')


    def due_pushed(self, checked = False):

        self.publisher_dict['xyz'].publish('Stanza 2')


    def charge_pushed(self, checked = False):

        self.publisher_dict['xyz'].publish('Ricarica')

    def rec_pushed(self, checked = False):

        self.publisher_dict['xyz'].publish('Reception')



    # def slider_activity(self):
    #     """
    #     Is called when user moves the slider. The set velocity
    #     is updated into text filed and private velocity variable.
    #     """
    #     self.update_velocity( self._widget.velocitySlider.value() )

