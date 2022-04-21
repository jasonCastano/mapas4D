#!/usr/bin/env python3
import serial
import sys
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import NavSatFix
import rospy
from nmea_msgs.msg import Sentence 
port = '/dev/ttyACM0'

#ser = serial.Serial(port, 230400, timeout=1.0)
ser = serial.Serial(port, 230400, timeout=1.0)
data = []

imu_pub = rospy.Publisher("imu/data_raw", Imu, queue_size=1)
nmea_pub = rospy.Publisher("nmea_sentence", Sentence, queue_size=1)
rospy.init_node("imu_node", anonymous=False)

grav = 0.0
n = 1;

gps_add_f = ['$GPGGA','$GLGGA','$GAGGA','$GNGGA']
gps_talker_id = {'$GP':1,'$GL':2,'$GN':4,'$GA':8}

def gps_decoder(gps_data):
    if(gps_data[0] in gps_add_f):
        latitud_raw = float(gps_data[2])
        longitud_raw = float(gps_data[4])
        altitud = float(gps_data[9])
        
        deg =  int(latitud_raw / 100)
        minutes = int(latitud_raw) % 100
        seg = int((latitud_raw*10000000) % 10000000)/10000000

        latitud = deg + minutes/60 + seg/60

        deg =  int(longitud_raw / 100)
        minutes = int(longitud_raw) % 100
        seg = int((longitud_raw*10000000) % 10000000)/10000000

        longitud = deg + minutes/60 + seg/60
        
        if(gps_data[3] == "S"):
            latitud = latitud * -1
        if(gps_data[5] == "W"):
            longitud = longitud * -1
        talker_id = gps_data[0][0] + gps_data[0][1] + gps_data[0][2] 
        return [gps_talker_id[talker_id], latitud, longitud, altitud]

    else:
        return 0



def read_data():
	error = True
	vec = []
	while (not(len(vec) >= 6 and error == False)):
		#print("En ciclo de lectura")
		try:
                    msg = ser.readline()	
                    msg_sensor = msg.decode('ascii')
                    vec = msg_sensor.split(",")
                    error = False
                    #print(vec)
		except UnicodeDecodeError:
                    error = True
	return vec,msg_sensor

while True:
    try:
        data,m_ = read_data()
        #print(data)
        
        if(len(data)==6 and not('$' in m_)):
            my_imu = Imu()
            my_imu.header.frame_id = "base_link"
            my_imu.orientation_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
            my_imu.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
            my_imu.linear_acceleration_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
        #Se hace cambio manual en las orientaciones debido a que estan organizadas diferentes al velodyne
        #y=x_ard;x=y_ard;z=z_ard;
            my_imu.angular_velocity.x = float(data[4])
            my_imu.angular_velocity.y = -float(data[3])
            my_imu.angular_velocity.z = float(data[5])
            my_imu.linear_acceleration.x = -float(data[1])
            my_imu.linear_acceleration.y = float(data[0])
            my_imu.linear_acceleration.z = -float(data[2])
            my_imu.header.stamp = rospy.Time.now()
            imu_pub.publish(my_imu)
        
        else:
            nmea_msg = Sentence()
            nmea_msg.sentence = m_
            nmea_msg.header.frame_id = "base_link"
            nmea_msg.header.stamp = rospy.Time.now()
            nmea_pub.publish(nmea_msg)


        #grav = grav + my_imu.linear_acceleration.z
        #print(grav/n)
        #n = n + 1
    except ValueError:
        continue


