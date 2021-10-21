#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include "ros/ros.h"


using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher my_imu_from_odom_pub;

sensor_msgs::Imu my_imu_odom;

void callback(const nav_msgs::Odometry::ConstPtr& odometry, const geometry_msgs::AccelWithCovarianceStamped::ConstPtr& accel)
{
  my_imu_odom.header = odometry->header;
  my_imu_odom.orientation = odometry->pose.pose.orientation;
  my_imu_odom.angular_velocity = odometry->twist.twist.angular;
  my_imu_odom.linear_acceleration = accel->accel.accel.linear;

  my_imu_from_odom_pub.publish(my_imu_odom); 
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_imu_from_odom_node");

  ros::NodeHandle nh;
  
  my_imu_odom.orientation_covariance = {0.001,0,0,0,0.001,0,0,0,0.001};
  my_imu_odom.angular_velocity_covariance = {0.001,0,0,0,0.001,0,0,0,0.001};
  my_imu_odom.linear_acceleration_covariance = {0.001,0,0,0,0.001,0,0,0,0.001};
  
  my_imu_from_odom_pub = nh.advertise<Imu>("my_imu_from_odom", 10);

  message_filters::Subscriber<nav_msgs::Odometry> odometry_sub(nh, "odometry/filtered/my_imu", 1);
  message_filters::Subscriber<geometry_msgs::AccelWithCovarianceStamped> accel_sub(nh, "accel/filtered/my_imu", 1);
  TimeSynchronizer<nav_msgs::Odometry, geometry_msgs::AccelWithCovarianceStamped> sync(odometry_sub, accel_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
