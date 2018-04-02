import RPi.GPIO as GPIO
import rospy
from std_msgs import Bool
from diagnostic_msgs import DiagnosticArray

CHANNEL =

def publish():
	leakPub = rospy.Publisher('Leak', Bool, queue_size=1)
	diagPub = rospy.Publisher('/diagnostic', DiagnosticArray)
	rospy.init_node('Leak')
	GPIO.setup(CHANNEL, GPIO.IN)
	freq = rospy.Rate(1)
	leak = Bool()
	pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
	diag = DiagnosticArray()
	while not rospy.is_shutdown():
		levelData=0
		if (leak.data):
			diag.status[0].level=2
		diag.status = [DiagnosticStatus(name='Leak', message=str(leak.data), level=levelData]
		leak.data = GPIO.input(CHANNEL)
		leakPub.publish(leak)
		diagPub.publish(diag)
		freq.sleep()

if __name__ == '__main__':
	try:
		publish()
	except rospy.ROSInterruptException:
		pass
