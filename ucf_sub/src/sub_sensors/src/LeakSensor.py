#!/usr/bin/env python
#import RPi.GPIO as GPIO
import gpio
import rospy
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

CHANNEL = 429 #should be 17

def publish():
	leakPub = rospy.Publisher('Leak', Bool, queue_size=1)
	diagPub = rospy.Publisher('/diagnostic', DiagnosticArray, queue_size=1)
	rospy.init_node('Leak')
	#GPIO.setmode(GPIO.BCM)
	#GPIO.setup(CHANNEL, GPIO.IN)
	GPIO.setup(CHANNEL, gpio.IN)
	freq = rospy.Rate(1)
	leak = Bool()
	pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
	diag = DiagnosticArray()
	while not rospy.is_shutdown():
		levelData=0
		if (leak.data):
			levelData=2
		leak.data = GPIO.input(CHANNEL)
		diag.status = [DiagnosticStatus(name='Leak', message=str(leak.data), level=levelData)]
		leakPub.publish(leak)
		diagPub.publish(diag)
		freq.sleep()

if __name__ == '__main__':
	try:
		publish()
	except rospy.ROSInterruptException:
		pass
