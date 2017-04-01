#!/usr/bin/env python
import roslib
import rospy
from geometry_msgs.msg import PoseStamped # for vrpn data
from geometry_msgs.msg import Twist  # for rc command
from sensor_msgs.msg import Image    	 # for receiving the video feed
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from drone_status import DroneStatus

from PySide import QtCore, QtGui
from ardrone_gui import *

import sys
import time,tf

CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 20 #ms

class ControlMainWindow(QtGui.QMainWindow):
    def __init__(self,parent = None):
        super(ControlMainWindow,self).__init__(parent)
        self.ui=Ui_MainWindow()
        self.ui.setupUi(self)

        #Qt signal & slot connection
        self.ui.btn_takeoff.clicked.connect(self.takeoff)
        self.ui.btn_land.clicked.connect(self.land)
        self.ui.btn_emergency.clicked.connect(self.emergency)
        self.ui.btn_switchmode.clicked.connect(self.switchmode)
        self.ui.sb_hoverpos_x.valueChanged.connect(self.hoverpos_x)
        self.ui.sb_hoverpos_y.valueChanged.connect(self.hoverpos_y)
        self.ui.sb_hoverpos_z.valueChanged.connect(self.hoverpos_z)

        #global variable
        self.ctrlmode = 1   # 0-manual, 1-automatic
        self.hoverpos = [-1500,500,1500]
        self.vrpnPos = [0,0,0,0,0,0]	# x,y,z,r,p,y from VRPN
        manualCmd = Twist()

        #ROS
        rospy.init_node('ardrone_ctrl', anonymous=True)
        self.subVideo   = rospy.Subscriber('/ardrone/image_raw',Image,self.ReceiveImage)
        self.subVrpn = rospy.Subscriber("/vrpn_client_node/drone1/pose", PoseStamped, self.ReceiveVrpndata)
        self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)
        self.pubCommand = rospy.Publisher('/cmd_vel',Twist)
        self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty)
        self.pubLand = rospy.Publisher('/ardrone/land',Empty)
        self.pubReset   = rospy.Publisher('/ardrone/reset',Empty)

        #Connection Timer
        self.isVrpnConnected = False
        self.VrpnConnectedSinceTimer = False
        self.isDroneConnected = False
        self.DroneConnectedSinceTimer = False
        self.connectionTimer = QtCore.QTimer(self)
        self.connectionTimer.timeout.connect(self.ConnectionCallback)
        self.connectionTimer.start(CONNECTION_CHECK_PERIOD)

# SUBSCRIBE CALLBACK
    def ReceiveVrpndata(self,data):
    	self.VrpnConnectedSinceTimer = True
    	#print data

    def ReceiveNavdata(self,data):
    	self.DroneConnectedSinceTimer = True
    	#print data

    def ReceiveImage(self,data):
    	pass

# TIMER CALLBACK
    def ConnectionCallback(self):
    	print 'Check Connection',self.isVrpnConnected,self.isDroneConnected
    	self.isVrpnConnected = self.VrpnConnectedSinceTimer
    	self.isDroneConnected = self.DroneConnectedSinceTimer
    	self.VrpnConnectedSinceTimer = False
    	self.DroneConnectedSinceTimer = False

# Commands
    def takeoff(self):
    	self.pubTakeoff.publish(Empty())
    	print 'CMD | Takeoff'

    def land(self):
    	self.pubLand.publish(Empty())
    	print 'CMD | Land'

    def emergency(self):
    	self.pubReset.publish(Empty())
    	print 'CMD | Emergency stop'
		#display.ui.btn_takeoff.setEnabled(False)

    def hoverpos_x(self,val):
    	self.hoverpos[0] = val
    	print self.hoverpos

    def hoverpos_y(self,val):
    	self.hoverpos[1] = val
    	print self.hoverpos

    def hoverpos_z(self,val):
    	self.hoverpos[2] = val
    	print self.hoverpos

    def switchmode(self):
    	if self.ctrlmode == 0:	# manual -> auto
    		self.ctrlmode = 1
    		self.ui.lb_mode.setText('AUTOMATIC')
    		print 'MODE | AUTO'
    	elif self.ctrlmode == 1:  # auto -> manual
    		self.ctrlmode = 0
    		self.ui.lb_mode.setText('MANUAL')
    		print 'MODE | MANUAL'

if __name__ == '__main__':
	app = QtGui.QApplication(sys.argv)
	display = ControlMainWindow()
	display.show()
	sys.exit(app.exec_())
