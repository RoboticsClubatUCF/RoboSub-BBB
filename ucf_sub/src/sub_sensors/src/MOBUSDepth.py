#!/usr/bin/env python


#!/usr/bin/env python
import ms5837
import rospy
from pymodbus.client.sync import ModbusSerialClient as modbus
from sensor_msgs import Temperature
import std_msgs import Float32

def publish():
	sensor = modbus(method='rtu', port='/dev/ttyUSB0', timeout=1, stopbits = 1, bytesize = 8,  parity='N', baudrate= 9600)
	sensor.connect()
	tempPub = rospy.Publisher('ExternalTemperature', Temperature, queue_size=1)
	depthPub = rospy.Publisher('Depth', Float32, queue_size=1)
	rospy.init_node('Depth')
	temp = Temperature()
	depth = Float32()
	freq = rospy.Rate(1)
	while not rospy.is_shutdown():
		rr = sensor.read_input_registers(address=1, count=2, unit=1);
		temp.temperature = rr.registers[1]
		depth.data = rr.registers[0]
		tempPub.publish(temp)
		depthPub.publish(depth)
		freq.sleep()

if __name__ == '__main__':
	try:
		publish()
	except rospy.ROSInterruptException:
		pass
