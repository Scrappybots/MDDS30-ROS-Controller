#!/usr/bin/env python3
# Written By: Bryan Ribas
# CC BY-SA 4.0

from lib2to3.pgen2.token import EQUAL
import rospy
import serial
from std_msgs.msg import Int16

def init():
    portname = rospy.get_param('~port')
    global serialConnection
    serialConnection = None

    # Connect to the serial port
    try:
        serialConnection = serial.Serial(
            port=portname,
            baudrate=9600,
            timeout=0.5
        )
    # Quit the node and print out error if connection fails
    except Exception as e:
        rospy.logerr("[MDDS30_controller]: Error connecting to MDDS30: " + str(e))
        rospy.signal_shutdown('Quit')  # Quit the node if it cannot connect

def mdds30_controller():

    # initialize the node 
    rospy.init_node('MDDS30_controller', anonymous=True)
    rospy.on_shutdown(shutdown)

    # connect to MDDS30
    init()

    # Setup the subscribers for each motor's speeds
    rospy.Subscriber('MDDS30_controller/motor_left',    Int16, motor_change, "left" )
    rospy.Subscriber('MDDS30_controller/motor_right',   Int16, motor_change, "right")

    rospy.spin()

def motor_change(speed,motor):
    global serialConnection
    packet = bytearray()
    byteSent = bytes()
    byteToSend = bytes()
    
    # We need to convert the -+100 to the byte expected
    # by the MDDS300. See the manual for more info:
    # http://www.tinyurl.com/3ddaevwa
 
    # First we check if the direction is CCW (positive) or CW (negative)
    if speed.data >= 0: # CCW motion
        if motor == "left":
            byteToSend = int((speed.data * 63) / 100) + 64
        else:
            byteToSend = int((speed.data * 63) / 100) + 192
    else: # CCW motion
        if motor == "left":
            byteToSend = int(((speed.data*-1) * 63) / 100) 
        else:
            byteToSend = int(((speed.data*-1) * 63) / 100) + 128
    
    # Only send if something changes
    if byteToSend != byteSent :
        packet.append(byteToSend)
        serialConnection.write(packet)
        packet.clear()
        byteSent = byteToSend
        # Let the log know what changed
        rospy.loginfo("[MDDS30_controller]: Motor {} Speed Changed To: {}".format(motor,speed))

def shutdown():
    global serialConnection
    rospy.loginfo("[MDDS30_controller]: shutting down!")
    # If the serial connection was open attempt to close it. 
    if serialConnection is not None:
        try:
            serialConnection.close
            rospy.loginfo("[MDDS30_controller]: Closed serial connection")
        except Exception as e:
            rospy.logerr("[MDDS30_controller]: Error disconnecting to MDDS30: " + str(e))        

if __name__ == '__main__':
    try:
        mdds30_controller()
    except rospy.ROSInterruptException as e:
        print(e)
        pass
