#include <string>
#include <iostream>
#include <cstdio>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <boost/asio.hpp>


class serial_node
{
public:
    serial_node(std::string port, unsigned int baud_rate) : io(), serial(io,port) //     SimpleSerial(std::string port, unsigned int baud_rate) : io(), serial(io,port)
    {
      node_.getParam("Device_Path", Device_Path);
      node_.getParam("Baud_Rate", Baud_Rate);
      node_.getParam("Toppic_For_Data_To_Be_Transmitted", Toppic_For_Data_To_Be_Transmitted);
      node_.getParam("Toppic_For_Recieved_Serial_Data", Toppic_For_Recieved_Serial_Data);

      // serial(io,Device_Path)baud_rate
      serial.set_option(boost::asio::serial_port_base::baud_rate(Baud_Rate));

      // Transmit_Sub_ = node_.subscribe<std_msgs::Int32>(Toppic_For_Data_To_Be_Transmitted, 100, &serial_node::Serial_Transmit_Callback, this);
      // Recieved_Pub_ = node_.advertise<std_msgs::String>(Toppic_For_Recieved_Serial_Data, 100, false);

      Transmit_Sub_ = node_.subscribe<std_msgs::Int32>("/TestIn", 100, &serial_node::Serial_Transmit_Callback, this);
      Recieved_Pub_ = node_.advertise<std_msgs::String>("/TestOut", 100, false);
    }

    /**
     * Write a string to the serial device.
     * \param s string to write
     * \throws boost::system::system_error on failure
     */
    void write(int Cmd);

    /**
     * Blocks until a line is received from the serial device.
     * Eventual '\n' or '\r\n' characters at the end of the string are removed.
     * \return a string containing the received line
     * \throws boost::system::system_error on failure
     */
    std::string readLine();
    ros::Publisher Recieved_Pub_;    //Publishes the recieved serial data
    std::string Toppic_For_Recieved_Serial_Data;

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
    std::string Device_Path;
    int Baud_Rate;

    // ROS
    ros::NodeHandle node_;

    ros::Subscriber Transmit_Sub_;  //Subscribes to serial topic to transmit the serial commands
    std::string Toppic_For_Data_To_Be_Transmitted;

    void Serial_Transmit_Callback(const std_msgs::Int32::ConstPtr& msg);
};
