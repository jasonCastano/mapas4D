source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
sudo chmod 777 /dev/ttyACM0
roslaunch velodyne_pointcloud VLP16_points.launch &
rosrun mapas4D sensor_imu_gps.py &
rosrun imu_complementary_filter complementary_filter_node &
rosrun nmea_navsat_driver nmea_topic_driver &
wait

