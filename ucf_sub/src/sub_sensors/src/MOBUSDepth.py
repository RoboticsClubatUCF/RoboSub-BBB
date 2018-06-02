#!/usr/bin/env python


#!/usr/bin/env python
import rospy
import struct
from pymodbus.client.sync import ModbusSerialClient as modbus
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float32

def publish():
	sensor = modbus(method='rtu', port='/dev/ttyUSB0', baudrate= 9600)
	sensor.connect()
	tempPub = rospy.Publisher('ExternalTemperature', Temperature, queue_size=1)
	depthPub = rospy.Publisher('Depth', Float32, queue_size=1)
	rospy.init_node('Depth')
	temp = Temperature()
	depth = Float32()
	freq = rospy.Rate(1)
	while not rospy.is_shutdown():
		rr = sensor.read_holding_registers(address=8, count=2, unit=1)
		temp.temperature = struct.unpack('>f',struct.pack('HH', *rr.registers)) 
		rr = sensor.read_holding_registers(address=2, count=2, unit=1)
		depth.data = struct.unpack('>f',struct.pack('HH', *rr.registers))
		tempPub.publish(temp)
		depthPub.publish(depth)
		freq.sleep()

if __name__ == '__main__':
	try:
		publish()
	except rospy.ROSInterruptException:
		pass
