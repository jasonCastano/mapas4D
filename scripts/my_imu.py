#!/usr/bin/env python3
import serial
import sys
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import rospy
port = '/dev/ttyACM0'

ser = serial.Serial(port, 9600, timeout=1.0)
data = []

imu_pub = rospy.Publisher("imu/data_raw", Imu, queue_size=10)

rospy.init_node("imu_node", anonymous=False)

grav = 0.0
n = 1;

def read_data():
	error = True
	vec = []
	while (not(len(vec) >= 6 and error == False)):
		#print("En ciclo de lectura")
		try:
			msg = ser.readline()	
			msg_sensor = msg.decode('ascii')
			vec = msg_sensor.split(",")
			#print(vec)
			error = False
		except UnicodeDecodeError:
			error = True
	return vec

while True:
    try:
        data = read_data()
        my_imu = Imu()
        my_imu.header.frame_id = "base_link"
        my_imu.orientation_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
        my_imu.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
        my_imu.linear_acceleration_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
        #Se hace cambio manual en las orientaciones debido a que estan organizadas diferentes al velodyne
        #y=x_ard;x=y_ard;z=z_ard;
        my_imu.angular_velocity.x = float(data[4])
        my_imu.angular_velocity.y = float(data[3])
        my_imu.angular_velocity.z = -float(data[5])
        my_imu.linear_acceleration.x = -float(data[1])
        my_imu.linear_acceleration.y = -float(data[0])
        my_imu.linear_acceleration.z = float(data[2])
        my_imu.header.stamp = rospy.Time.now()
        imu_pub.publish(my_imu)
        #grav = grav + my_imu.linear_acceleration.z
        #print(grav/n)
        #n = n + 1
    except ValueError:
        continue


