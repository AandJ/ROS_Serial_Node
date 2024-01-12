/***

 */
#include "serial_node/int_serial_node.hpp"

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

using namespace std;
// using namespace boost;

void serial_node::Serial_Transmit_Callback(const std_msgs::Int32::ConstPtr& msg)
{
  int Val = msg->data;
  write(Val);
  ROS_INFO("I wrote: [%d]", Val);
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


void serial_node::write(int Cmd)
{


  char array[6];
  array[0] = 0x80 | (Cmd & 0x0000007F);            // Least significant 7 bits
  array[1] = 0x80 | (Cmd & 0x00003F80) >> 7;       // bits 8 through 14
  array[2] = 0x80 | (Cmd & 0x001FC000) >> 14;      // bits 15 through 21
  array[3] = 0x80 | (Cmd & 0x0FE00000) >> 21;      // bits 22 through 28
  array[4] = 0x80 | (Cmd & 0xF0000000) >> 28;      // bits 29 through 32

  array[5] = 0x0D;

  boost::asio::write(serial,boost::asio::buffer(array,sizeof(array)));


    // char array_1[2];
    // char array_2[2];
    // char array_3[2];
    //
    // array_1[0] = (Cmd & 0x000000FF);
    // array_1[1] = (Cmd & 0x0000FF00) >> 8;
    // array_2[0] = (Cmd & 0x00FF0000) >> 16;
    // array_2[1] = (Cmd & 0xFF000000) >> 24;
    // array_3[0] = 0x0D;
    // array_3[1] = 0;
    //
    // // boost::asio::write(serial,boost::asio::buffer(array_1,sizeof(array_1)));
    // boost::asio::write(serial,boost::asio::buffer(array_2,sizeof(array_2)));
    // boost::asio::write(serial,boost::asio::buffer(array_3,sizeof(array_3)));

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

      // std::stringstream ss;
      //
      // // Listen for a string, terminated by a new line, and save to the string stream ss
      // ss << serial.readLine();
      //
      // // set data of ROS msg to the recieved string
      // msg.data = ss.str();
      // // ROS_INFO("%s", msg.data.c_str());
      //
      // //  Publish the ROS msg with recieved string
      // serial.Recieved_Pub_.publish(msg);
      // ros::spinOnce();

      ros::spin();

      // loop_rate.sleep();

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
