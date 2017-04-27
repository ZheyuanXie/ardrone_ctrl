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

from threading import Lock
import sys
import time,tf

CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 20 #ms

class ControlMainWindow(QtGui.QMainWindow):
    vrpndataArrive = QtCore.Signal()
    dronedataArrive = QtCore.Signal()
    def __init__(self,parent = None):
        super(ControlMainWindow,self).__init__(parent)
        self.ui=Ui_MainWindow()
        self.ui.setupUi(self)
        self.uilock = QtCore.QMutex()

        #Qt signal & slot connection

        self.vrpndataArrive.connect(self.UpdateVrpndata)
        self.dronedataArrive.connect(self.UpdataDronedata)
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
        self.droneVel = [0,0,0]
        self.image = None
        self.imageLock = Lock()
        self.manualCmd = Twist()

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
        self.vrpnPos[0] = data.pose.position.x * 1000
        self.vrpnPos[1] = data.pose.position.y * 1000
        self.vrpnPos[2] = data.pose.position.z * 1000
        quaternion = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.vrpnPos[3] = euler[0]
        self.vrpnPos[4] = euler[1]
        self.vrpnPos[5] = euler[2]
        self.vrpndataArrive.emit()

        #generate automatic control
        ex = self.vrpnPos[0] - self.hoverpos[0]
        ey = self.vrpnPos[2] - self.hoverpos[2]
        kp = -1/2000.0
        px = ex * kp
        py = ey * kp
        kd = 1/3000.0
        t = Twist()
        t.linear.x  = py - kd * self.droneVel[0] # pitch
        t.linear.y  = px - kd * self.droneVel[1] # roll
        t.angular.z = -self.vrpnPos[4] # yaw
        self.pubCommand.publish(t)

    def UpdateVrpndata(self):
        self.ui.lb_vrpn_px.setText('%.3f'%self.vrpnPos[0])
        self.ui.lb_vrpn_py.setText('%.3f'%self.vrpnPos[1])
        self.ui.lb_vrpn_pz.setText('%.3f'%self.vrpnPos[2])
        self.ui.lb_vrpn_ox.setText('%.3f'%self.vrpnPos[3])
        self.ui.lb_vrpn_oy.setText('%.3f'%self.vrpnPos[4])
        self.ui.lb_vrpn_oz.setText('%.3f'%self.vrpnPos[5])

    def ReceiveNavdata(self,data):
    	self.DroneConnectedSinceTimer = True
        self.droneVel[0] = data.vx
        self.droneVel[1] = data.vy
        self.droneVel[2] = data.vz
        self.dronedataArrive.emit()

    def UpdataDronedata(self):
        self.ui.lb_navdata_vx.setText('%.3f'%self.droneVel[0])
        self.ui.lb_navdata_vy.setText('%.3f'%self.droneVel[1])
        self.ui.lb_navdata_vz.setText('%.3f'%self.droneVel[2])

    def ReceiveImage(self,data):
    	self.imageLock.acquire()
        try:
            self.image = data
        finally:
            self.imageLock.release()


# TIMER CALLBACK
    def ConnectionCallback(self):
    	# print 'Check Connection',self.isVrpnConnected,self.isDroneConnected
    	self.isVrpnConnected = self.VrpnConnectedSinceTimer
    	self.isDroneConnected = self.DroneConnectedSinceTimer
    	self.VrpnConnectedSinceTimer = False
    	self.DroneConnectedSinceTimer = False
        self.statusBar().showMessage('VRPN:%s | DRONE:%s'%(str(self.isVrpnConnected),str(self.isDroneConnected)))


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
