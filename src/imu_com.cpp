#include <boost/asio.hpp>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>


using namespace std;
using namespace boost;

// referencia: https://gist.github.com/kaliatech/427d57cb1a8e9a8815894413be337cf9

class SimpleSerial
{
public:
    /**
     * Constructor.
     * \param port device name, example "/dev/ttyUSB0" or "COM4"
     * \param baud_rate communication speed, example 9600 or 115200
     * \throws boost::system::system_error if cannot open the
     * serial device
     */
    SimpleSerial(std::string port, unsigned int baud_rate)
    : io(), serial(io,port)
    {
        serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    }

    /**
     * Write a string to the serial device.
     * \param s string to write
     * \throws boost::system::system_error on failure
     */
    void writeString(std::string s)
    {
        boost::asio::write(serial,boost::asio::buffer(s.c_str(),s.size()));
    }

    /**
     * Blocks until a line is received from the serial device.
     * Eventual '\n' or '\r\n' characters at the end of the string are removed.
     * \return a string containing the received line
     * \throws boost::system::system_error on failure
     */
    std::string readLine()
    {
        //Reading data char by char, code is optimized for simplicity, not speed
        using namespace boost;
        char c;
        std::string result;
        for(;;)
        {
            asio::read(serial,asio::buffer(&c,1));
            //std::cout << c << std::endl;
            switch(c)
            {
                case '\r':
                    break;
                case '\n':
                    return result;
                default:
                    result+=c;
            }
        }
    }

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
};


std::vector<double> splitString(std::string str){
    
    vector<double> v;
    std::stringstream ss(str);
    while(ss.good()){
        try{
            std::string substr;
            std::getline(ss, substr, ',');
            v.push_back(std::stod(substr));
        }
        catch(boost::system::system_error& e){
            std::cout<<"Error: "<<e.what()<<std::endl;
            return v;
        }
    }

    return v;
    
}

int main(int argc, char* argv[])
{
    try {

        ros::init(argc, argv, "imu_com_node");
        ros::NodeHandle n;
        ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);

        std::vector<double> data;
        std::string rawData = "";
        SimpleSerial serial("/dev/ttyACM0",115200);
        while(1){
            rawData = serial.readLine();
            //std::cout << rawData << std::endl;
            data = splitString(rawData);
            if(data.size() == 6){
                sensor_msgs::Imu my_imu;
                my_imu.header.frame_id = "base_link";
                my_imu.orientation_covariance = {0.001, 0.0,0.0,0.0,0.001,0.0,0.0,0.0,0.001};
                my_imu.angular_velocity_covariance = {0.001, 0.0,0.0,0.0,0.001,0.0,0.0,0.0,0.001};
                my_imu.linear_acceleration_covariance = {0.001, 0.0,0.0,0.0,0.001,0.0,0.0,0.0,0.001};

                my_imu.angular_velocity.x = data[4];
                my_imu.angular_velocity.y = data[3];
                my_imu.angular_velocity.z = -data[5];
                my_imu.linear_acceleration.x = -data[1];
                my_imu.linear_acceleration.y = -data[0];
                my_imu.linear_acceleration.z = data[2];

                my_imu.header.stamp = ros::Time::now();

                imu_pub.publish(my_imu);
            
            }
        }
    } catch(boost::system::system_error& e)
    {
        cout<<"Error: "<<e.what()<<endl;
        return 1;
    }
}


