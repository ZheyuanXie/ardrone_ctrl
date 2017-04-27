#!/usr/bin/env python
import rospy
from drone_controller import BasicDroneController
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist  
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from drone_status import DroneStatus
import time, sys
import tf


controller = BasicDroneController() #ms
pose = [0,0,0,0,0,0]
lpose = [0,0,0,0,0,0]
vel = [0,0]
hoverPose = [-1500, 500, 1500]
StatusMessages = {
        -1 : 'error',
        0 : 'Emergency',
        1    : 'Initialized',
        2    : 'Landed',
        3    : 'Flying',
        4  : 'Hovering',
        5      : 'Test (?)',
        6 : 'Taking Off',
        7 : 'Going to Hover Mode',
        8   : 'Landing',
        9   : 'Looping (?)'
        }

def pctrl(e):
    output = -e / 2000.0
    sat = 0.15
    if output > sat:
        output = sat
    if output < -sat:
        output = -sat
    return output

def vrpn_callback(data):
    lpose = pose[:]
    #lts = ts[:]
    pose[0] = data.pose.position.x * 1000
    pose[1] = data.pose.position.y * 1000
    pose[2] = data.pose.position.z * 1000
    quaternion = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    pose[3] = euler[0]
    pose[4] = euler[1]
    pose[5] = euler[2]
    ex = pose[0] - hoverPose[0]
    ey = pose[2] - hoverPose[2]
   # controller.SetCommand(pctrl(ex), pctrl(ey), 0, 0)
    t = Twist()
    kd = 1/3000.0
    t.linear.x  = pctrl(ey) + kd * vel[0] # pitch
    t.linear.y  = pctrl(ex) - kd * vel[1] # roll
    t.angular.z = -pose[4]

    #print t.linear.z
    #print 'x:%.3f,%.3f,%.3f\ny:%.3f,%.3f,%.3f'%(ex,pctrl(ex),kd*vel[0],ey,pctrl(ey),kd * vel[0])
    #controller.SendCommand()
    #f.writelines('%s, %f, %f\n'%(str(pose),pctrl(ex),pctrl(ey)))
    #print 'status:%s,rc_rpy:%.3f,%.3f'%(StatusMessages[controller.status],pctrl(ex),pctrl(ey))


def navdata_callback(data):
    vel[0] = data.vx   #pitch
    vel[1] = data.vy   #roll
    #print 'vel:',data.vx,data.vy

    pass  # currently we don't need to handle any data from drones

def onClose():
    f.close()


if __name__ == '__main__':
    rospy.init_node('ardrone_ctrl', anonymous=True)
    rospy.Subscriber("/vrpn_client_node/drone1/pose", PoseStamped, vrpn_callback)
    rospy.Subscriber('/ardrone/navdata',Navdata,navdata_callback)
    pubCommand = rospy.Publisher('/cmd_vel',Twist)
    rospy.on_shutdown(onClose)

    # create a file for flight data recording
    t = str(time.time())
    print('time %s'%t)
    f = open('/home/fdsa/%s.txt'%t,'w')
    rospy.sleep(2.)
    # takeoff the drone
    if controller.status == 0:
        controller.SendEmergency()
    rospy.sleep(2.)
    print 'TAKEOFF\a'
    controller.SendTakeoff()
    rospy.sleep(2.)

    rospy.spin()
