/***

 */
#include "serial_node/serial_node.hpp"

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

using namespace std;
// using namespace boost;

void serial_node::Serial_Transmit_Callback(const std_msgs::String::ConstPtr& msg)
{
  std::string s = msg->data.c_str();
  writeString(s);
  // ROS_INFO("I heard: [%s]", s);
}

std::string serial_node::readLine()
{
    //Reading data char by char, code is optimized for simplicity, not speed
    using namespace boost;
    char c;
    std::string result;
    for(;;)
    {
        asio::read(serial,asio::buffer(&c,1));
        switch(c)
        {
            // Ignore Carriage Return
            case '\r':
                break;
            // If newline the return the recieved command
            case '\n':
                return result;
            default:
                result+=c;
        }
    }
}


void serial_node::writeString(std::string s)
{
    boost::asio::write(serial,boost::asio::buffer(s.c_str(),s.size()));
}





/**
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  serial_node serial("/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066FFF303555483043224509-if02",9600);

  std_msgs::String msg;
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    try
    {

      std::stringstream ss;

      // Listen for a string, terminated by a new line, and save to the string stream ss
      ss << serial.readLine();

      // set data of ROS msg to the recieved string
      msg.data = ss.str();
      // ROS_INFO("%s", msg.data.c_str());

      //  Publish the ROS msg with recieved string
      serial.Recieved_Pub_.publish(msg);
      ros::spinOnce();

      loop_rate.sleep();

    }
    catch(boost::system::system_error& e)
    {
        // ERROR condition for access to the serial port
        std::stringstream ss;
        ss << "Error: " << e.what() << endl;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        return 1;
    }
  }


  return 0;
}
