#!/usr/bin/env python

import roslib #needed for rviz component
import rospy #needed for communication with ROS

import sys
import rosservice
from std_msgs.msg import String #more imports needed for ros communication
from std_msgs.msg import Bool
from std_msgs.msg import Int16

from binpicking_arduino.srv import isButtonPressed, isButtonPressedRequest, isButtonPressedResponse #arduino button syngery imports

from python_qt_binding.QtWidgets import * #needed QT gui imports
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *
from PyQt5 import QtTest

import rviz ## Finally import the RViz bindings themselves.
import getpass

user = getpass.getuser()

class TopicFetcher(QObject): # Thread retreiving all currently active topics and services, once per second.
    
    topic_signal = pyqtSignal(str)

    pyqtSlot()
    def monitor_topics(self):
        
        while True:
            topics = rospy.get_published_topics()
            topic_string = str(topics)
            service_list = rosservice.get_service_list()
            service_string = str(service_list)
            topics_and_services = topic_string + service_string
            self.topic_signal.emit(topics_and_services)  #push list to callback
            QtTest.QTest.qWait(1000)



class MainWindow(QMainWindow): 
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent) #creating mainwindow, placeholder for the two potential screens
        self.central_widget = QStackedWidget()
        self.setCentralWidget(self.central_widget)

        self.start_screen = StartScreen()
        self.central_widget.addWidget(self.start_screen)
        self.central_widget.setCurrentWidget(self.start_screen) #
        self.start_screen.start_button.clicked.connect(self.start_binPicking)
        self.start_screen.calibrate_button.clicked.connect(self.start_calibrate)
        self.binpicking_screen = BinpickingScreen()
        self.central_widget.addWidget(self.binpicking_screen)
        self.binpicking_screen.stop_button.clicked.connect(self.stop_binPicking) #creation of start and binpicking screen, setting startscreen in the mainwindow.

        self.topic_fetcher = TopicFetcher() #creation and starting of topic_fetching thread
        self.thread = QThread(self)
        self.topic_fetcher.topic_signal.connect(self.topic_callback)
        self.topic_fetcher.moveToThread(self.thread)
        self.thread.started.connect(self.topic_fetcher.monitor_topics)
        self.thread.start()  

        self.camera1_connected = False #list of booleans for gui logic
        self.camera2_connected = False
        self.dobot_connected = False
        #self.arduino_connected = False
        self.binpickingStateInGUI = False
        self.isCalibrating = False
        self.isButtonPressed = False
        self.idleHasStarted = False
        self.isReconnecting = False

        rospy.wait_for_service('binpicking/button') #waiting for button service to be available
        self.buttonCheck = rospy.ServiceProxy('binpicking/button', isButtonPressed)


    pyqtSlot(str)
    def topic_callback(self, topicsAndServices):
        
        res = self.buttonCheck()   #handeling buttonlogic via button topic, will change screen whenever physical button or arduino button is pushed.
        if not res.isPressed == self.isButtonPressed:
            self.isButtonPressed = res.isPressed
            if self.isButtonPressed:
                self.binpickingStateInGUI = True
                rospy.loginfo("GUI Received start binpicking")
                self.central_widget.setCurrentWidget(self.binpicking_screen)
            else:
                self.binpickingStateInGUI = False
                rospy.loginfo("GUI Received stop binpicking")
                self.central_widget.setCurrentWidget(self.start_screen)

        camera1_available = "/camera1/depth/color/points"  #topics and service to check for availability
        camera2_available = "/camera2/depth/color/points"
        magician_available = "magician/moveToPosition"
        #arduino_available = "arduino_toggle_led"

        anythingOffline = not self.camera1_connected or not self.camera2_connected or not self.dobot_connected #or not self.arduino_connected

        if camera1_available in topicsAndServices:  #checklist for availble topics, if available, device = connected.
            self.camera1_connected = True
            self.set_status_connected(self.start_screen.camera1_status)
        else:
            self.camera1_connected = False
            self.set_status_disconnected(self.start_screen.camera1_status)
        if camera2_available in topicsAndServices:
            self.camera2_connected = True
            self.set_status_connected(self.start_screen.camera2_status)
        else:
            self.camera2_connected = False
            self.set_status_disconnected(self.start_screen.camera2_status)
        if magician_available in topicsAndServices and not self.isReconnecting:
            self.dobot_connected = True
            self.set_status_connected(self.start_screen.dobot_status)
        else:
            self.dobot_connected = False
            self.set_status_disconnected(self.start_screen.dobot_status)
        #if arduino_available in topicsAndServices:
        #    self.arduino_connected = True
        #    self.set_status_connected(self.start_screen.arduino_status)
        #else:
        #    self.arduino_connected = False 
        #    self.set_status_disconnected(self.start_screen.arduino_status)
        #if self.arduino_connected and self.dobot_connected and self.camera1_connected and self.camera2_connected: 
        if self.dobot_connected and self.camera1_connected and self.camera2_connected: 
            if not self.isCalibrating:
                self.enable_start_button()
                self.start_screen.calibrate_button.setEnabled(True) #enable start binpicking button if all devices are connected(and not calibrating)
        elif anythingOffline:
            self.disable_start_button()
            if self.binpickingStateInGUI: #whenver any device is offline snap out of binpicking and disable start button
                self.stop_binPicking()
        if self.isCalibrating or not self.idleHasStarted:
            self.disable_start_button()
            self.start_screen.calibrate_button.setEnabled(False)
        
    def start_binPicking(self): #method linked to startbinpicking button, will publish true to button topick to switch screen.
        self.binpickingStateInGUI = True
        self.pub.publish(True)
    
    def stop_binPicking(self): #method linked to stop binpicking button
        self.binpickingStateInGUI = False
        self.pub.publish(True)

    def set_status_connected(self, qlabel):
        connected_sheet = 'font: Bold;font-size: 60px; font-family: Monospace; color: green;'
        qlabel.setText("CONNECTED")
        qlabel.setStyleSheet(connected_sheet)
    
    def set_status_disconnected(self, qlabel):
        disconnected_sheet = 'font: Bold;font-size: 60px; font-family: Monospace; color: red;'
        qlabel.setText("DISCONNECTED")
        qlabel.setStyleSheet(disconnected_sheet)

    def disable_start_button(self):
        self.start_screen.start_button.setEnabled(False)
        self.start_screen.start_button.setStyleSheet("""
                border: 5px solid black;
                border-radius: 150px;
                color: white;
                font-size: 112px;
                background-color: grey;
                height: 450px;""")

    def enable_start_button(self):
        self.start_screen.start_button.setEnabled(True)
        self.start_screen.start_button.setStyleSheet("""
                border: 5px solid black;
                border-radius: 150px;
                color: white;
                font-size: 112px;
                background-color: #4CAF50;
                height: 450px;""")            
    
    def start_calibrate(self): #method linked to calibrate button push
        self.calibrate.publish(True)

    def initCommunicator(self): #initialize ros communication node
        rospy.init_node('binpicking_gui_node', anonymous=True)
        rospy.loginfo("Rosnode for ros communication in GUI initialised")
        rospy.Subscriber('flexbe/behavior_update', String, self.callbackStateMachine) #fetch updates from flexbe behaviours
        rospy.loginfo("GUI subscribed to flexbe/behavior_update")

        self.pub = rospy.Publisher('button_pushed', Bool,queue_size=10) # create publisher for button pressed signal
        rospy.loginfo("Started button_pushed subscriber in GUI")
        self.calibrate = rospy.Publisher('calibrate', Bool,queue_size=10) # create publisher for calibration method
        rospy.loginfo("Started calibrate subscriber in GUI")        

    def callbackStateMachine(self, data): #callback handling from flexbe behaviour updates topic        
        stateTextarray = data.data.split('/')
        stateText = stateTextarray[len(stateTextarray) - 1]
        rospy.loginfo(data.data)
        if data.data == "/Reconnect":
            self.isReconnecting = True
        if data.data == "/idle":
            self.idleHasStarted = True
        if data.data == "/calibrate":
            self.isCalibrating = True
            self.isReconnecting = False
        else:
            self.isCalibrating = False
        text = "Current State:\t%s" % stateText
        self.binpicking_screen.current_state.setText(text) #show robots current state in binpicking screen
           
class StartScreen(QWidget):
    def __init__(self, parent=None):
        super(StartScreen, self).__init__(parent)
        layout = QHBoxLayout()#complete layout startscreen
        layoutLeft = QVBoxLayout() #left side (images + status labels)
        layoutRight = QVBoxLayout() #right side(start + call button)
        LayoutStatus = QHBoxLayout() #layout container for status labels
        LayoutStatusLeft = QVBoxLayout() #layout for device names
        LayoutStatusRight = QVBoxLayout() #layout for device statusus
        layout.addLayout(layoutLeft)
        
        self.avans_pic = QLabel()     #avans image label creation
        self.avans_pic.setContentsMargins(175,0,0,0)
        p = QPixmap('/home/' + user + '/BinPicking/src/binpicking/binpicking_app/src/Avans.png')
        size = QSize(600,1200)
        self.avans_pic.setPixmap(p.scaled(size,Qt.KeepAspectRatio))   
        
        self.sr_pic = QLabel()        #smart robotics image label creation
        self.sr_pic.setContentsMargins(175,0,0,0)
        p = QPixmap('/home/' + user + '/BinPicking/src/binpicking/binpicking_app/src/smartrobotics.png')
        size = QSize(600,1200)
        self.sr_pic.setPixmap(p.scaled(size,Qt.KeepAspectRatio))

        layoutLeft.addLayout(LayoutStatus)  
        LayoutStatus.addLayout(LayoutStatusRight)
        LayoutStatus.addLayout(LayoutStatusLeft) 
        
        self.camera1_status = QLabel("DISCONNECTED")
        self.camera2_status = QLabel("DISCONNECTED")
        self.dobot_status = QLabel("DISCONNECTED")
        #self.arduino_status = QLabel("DISCONNECTED")
        
        disconnected_sheet = 'font: Bold;font-size: 60px; font-family: Monospace; color: red;'
        self.camera1_status.setStyleSheet(disconnected_sheet)
        self.camera2_status.setStyleSheet(disconnected_sheet)
        self.dobot_status.setStyleSheet(disconnected_sheet)
        #self.arduino_status.setStyleSheet(disconnected_sheet)

        self.camera1_label = QLabel("Camera 1:")
        self.camera2_label = QLabel("Camera 2:")
        self.dobot_label = QLabel("Dobot:")
        #self.arduino_label = QLabel("Arduino:")

        self.camera1_label.setContentsMargins(30,0,0,0)
        self.camera2_label.setContentsMargins(30,0,0,0)
        self.dobot_label.setContentsMargins(30,0,0,0)
        #self.arduino_label.setContentsMargins(30,0,0,0)

        label_sheet = 'font: Bold;font-size: 60px; font-family: Monospace; color: white;'
        self.camera1_label.setStyleSheet(label_sheet)
        self.camera2_label.setStyleSheet(label_sheet)
        self.dobot_label.setStyleSheet(label_sheet)
        #self.arduino_label.setStyleSheet(label_sheet)

        LayoutStatusRight.addWidget(self.dobot_label)
        LayoutStatusRight.addWidget(self.camera1_label)
        LayoutStatusRight.addWidget(self.camera2_label)
        #LayoutStatusRight.addWidget(self.arduino_label)

        LayoutStatusLeft.addWidget(self.dobot_status)
        LayoutStatusLeft.addWidget(self.camera1_status)
        LayoutStatusLeft.addWidget(self.camera2_status)
        #LayoutStatusLeft.addWidget(self.arduino_status)

        layout_images = QVBoxLayout()
        layoutLeft.addLayout(layout_images)
        layout_images.addWidget(self.sr_pic)
        layout_images.addWidget(self.avans_pic)     #bunch of adding labels to the left side of main screen

        layout.addLayout(layoutRight)
        self.calibrate_button = QPushButton('Calibrate')
        self.calibrate_button.setStyleSheet("""
            border: 5px solid black;
            border-radius: 150px;
            color: white;
            font-size: 112px;
            background-color: #008CBA;
            height: 450px; 
        """)
        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(20)
        self.calibrate_button.setGraphicsEffect(shadow)

        self.start_button = QPushButton('Start Binpicking')
        self.start_button.setEnabled(False)
        self.start_button.setStyleSheet("""
            border: 5px solid black;
            border-radius: 150px;
            color: white;
            font-size: 112px;
            background-color: grey;
            height: 450px; 
        """)
        shadow2 = QGraphicsDropShadowEffect()
        shadow2.setBlurRadius(20)  
        self.start_button.setGraphicsEffect(shadow2)

        layoutRight.addWidget(self.calibrate_button)
        layoutRight.addWidget(self.start_button)   # added start and calibrate button to main screen
        self.setLayout(layout)   #set mainlayout startscreen

class BinpickingScreen( QWidget ): #class to create the binpicking screen with rviz component
    def __init__(self):
        QWidget.__init__(self)
        self.frame = rviz.VisualizationFrame() #creation of the rviz frame in screen.
        self.frame.initialize()

        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config,"/home/" + user + "/BinPicking/src/binpicking/binpicking_launch/rviz/config.rviz" )
        self.frame.load(config)

        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( False ) 
        self.manager = self.frame.getManager() 

        layout_complete_bpc = QVBoxLayout() # layout container binpicking screen

        self.stop_button = QPushButton('Stop Binpicking') #creation of stop button
        self.stop_button.setStyleSheet("""
            border: 5px solid black;
            border-radius: 25px;
            color: white;
            font-size: 30px;
            background-color: rgb(255, 0, 0);
            height: 100px; """)   

        self.current_state = QLabel("Current State:\n\NO STATE FOUND") #creation current state label
        self.current_state.setStyleSheet('font-size: 40px; font: Bold; color: white;')
        self.current_state.setGeometry(QRect(0, 0, 300, 100)) 
        
        layout_complete_bpc.addWidget(self.current_state)
        layout_complete_bpc.addWidget(self.frame )  
        layout_complete_bpc.addWidget(self.stop_button)   #adding created components to layout

        self.setLayout( layout_complete_bpc )  

#adding wallpaper image to the mainwindow
stylesheet = """  
    QMainWindow {{
        background-image: url(/home/{0}/BinPicking/src/binpicking/binpicking_app/src/wallpaper4.jpg); 
        background-repeat: no-repeat; 
        background-position: center;
    }}
"""
stylesheet = stylesheet.format(user)

if __name__ == '__main__': 
    app = QApplication([]) #initialising app
    app.setStyleSheet(stylesheet) #setting wallpaper image
    window = MainWindow() #creating mainwindow container for start / binpicking screen
    window.showFullScreen()#start app in fullscreen
    try:   #try to launch ros communication node
        window.initCommunicator()
    except rospy.ROSInterruptException:
        pass
    sys.exit(app.exec_()) #needed to be able to close GUI.
