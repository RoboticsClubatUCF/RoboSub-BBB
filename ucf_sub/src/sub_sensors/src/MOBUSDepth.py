#!/usr/bin/env python3
import rospy
import struct
from pymodbus.client.sync import ModbusSerialClient as modbus
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped

def publish():
	sensor = modbus(method='rtu', port='/dev/ucfsub/depth', baudrate=115200)
	sensor.connect()
	tempPub = rospy.Publisher('ExternalTemperature', Temperature, queue_size=1)
	depthPub = rospy.Publisher('depth', Float32, queue_size=1)
	posePub = rospy.Publisher('/depth/pose', PoseWithCovarianceStamped, queue_size=1)
	rospy.init_node('Depth')
	temp = Temperature()
	depth = Float32()
	
	freq = rospy.Rate(20)
	loop = 0
	
	pose = PoseWithCovarianceStamped()
	pose.header.frame_id = "odom"
	pose.pose.covariance = [0.0]*36
	pose.pose.covariance[14] = 0.01
	pose.pose.pose.orientation.x = 0.0
	pose.pose.pose.orientation.y = 0.0
	pose.pose.pose.orientation.z = 0.0
	pose.pose.pose.orientation.w = 1.0
	pose.pose.pose.position.x = 0.0
	pose.pose.pose.position.y = 0.0
        
	while not rospy.is_shutdown():
		if loop >= 20:
			rr = sensor.read_holding_registers(address=8, count=2, unit=1)
			temp.temperature = struct.unpack('>f',struct.pack('>HH', *rr.registers))[0]
			tempPub.publish(temp)
			loop = 0
		loop += 1
			
		rr = sensor.read_holding_registers(address=2, count=2, unit=1)
		depth.data = struct.unpack('>f',struct.pack('>HH', *rr.registers))[0]
		depthPub.publish(depth)

		pose.pose.pose.position.z = depth.data * 10.2
		pose.header.stamp = rospy.Time.now()
		posePub.publish(pose)
		freq.sleep()

if __name__ == '__main__':
	try:
		publish()
	except rospy.ROSInterruptException:
		pass
