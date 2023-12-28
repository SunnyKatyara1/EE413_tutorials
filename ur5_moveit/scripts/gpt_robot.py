#!/usr/bin/env python3
import os
import rospy
import rospkg
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLineEdit

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        self.setObjectName('MyPlugin')

        # Create QWidget
        self._widget = QWidget()
        # Extend the widget with all attributes and children from UI file
        # (loadUi is not necessary if you create all widgets in the code)
        # ui_file = os.path.join(rospkg.RosPack().get_path('my_rqt_plugin'), 'resource', 'MyPlugin.ui')
        # loadUi(ui_file, self._widget)

        # Create layout and widgets
        layout = QVBoxLayout()
        self.name_entry = QLineEdit(self._widget)
        imr_api_button = QPushButton('IMr API', self._widget)
        imr_chatbot_button = QPushButton('IMR Chatbot', self._widget)

        # Connect callbacks
        self.name_entry.returnPressed.connect(self.on_enter_key)
        imr_api_button.clicked.connect(self.imr_api_button_clicked)
        imr_chatbot_button.clicked.connect(self.imr_chatbot_button_clicked)

        # Add widgets to layout
        layout.addWidget(self.name_entry)
        layout.addWidget(imr_api_button)
        layout.addWidget(imr_chatbot_button)

        # Set layout to widget
        self._widget.setLayout(layout)

        # Add widget to the user interface
        context.add_widget(self._widget)

    def on_enter_key(self):
        # Handle enter key press
        name = self.name_entry.text()
        # Call your UR5 drawing function here
        rospy.loginfo('Drawing name: {}'.format(name))

    def imr_api_button_clicked(self):
        # Handle IMr API button click
        rospy.loginfo('IMR API button clicked')

    def imr_chatbot_button_clicked(self):
        # Handle IMR Chatbot button click
        rospy.loginfo('IMR Chatbot button clicked')
