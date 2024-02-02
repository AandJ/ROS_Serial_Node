// Include standard libraries
#include <iostream>
#include <ctime>
#include <cstdio>
#include <sstream>
#include <string>

// Include ROS headers
#include "ros/ros.h"
#include "std_msgs/String.h"

// Include Boosts headers
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// Include Point Cloud Libraries (PCL)
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>  // Allows subscription to ROS topic with pcl::PointCloud<PointT> data types


// Include my serialisation header for serialisation and de-serialisation functions
#include "serial_node/pointcloud_processing.hpp"
#include "serial_node/boost_comms.hpp"


using boost::asio::ip::udp;
using namespace std;
using namespace ros;
using namespace Eigen;

enum message_stage {info, data};

// Define objects and variables for ROS communication
pcl::PCLPointCloud2 Output_cloud; // the reconstructed cloud to be output to the new ROS master
Publisher point_cloud_publisher_;

// Variables to store server parameters
bool Debug_Mode;
std::string output_topic;
int Output_Buffer_Length;

// Server comm defintions
boost::asio::io_context io_context;
udp::socket socket_(io_context, udp::endpoint(udp::v4(), 25000));
udp::endpoint remote_endpoint_;
boost::array<char, 65000> recv_buffer_; // Max is ~ 65000

message_stage expected_message_stage;
int recv_data_length_int;


// UDP server function definitions
void start_receive();

void handle_receive(const boost::system::error_code& error,
    std::size_t bytes_transferred);

void handle_send(boost::shared_ptr<std::string> /*message*/,
      const boost::system::error_code& /*error*/,
      std::size_t /*bytes_transferred*/);

/* Function to listens on specified port for recieved point clouds */
void start_receive()
{
    socket_.async_receive_from(
        boost::asio::buffer(recv_buffer_), remote_endpoint_,
        boost::bind(&handle_receive,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
}



/* Function is called when a point cloud is recieved */
void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if (!error)
    {
      std::string Confirm_reciept_str;

      if(expected_message_stage == info) {
          // First check that the recieved msgs is the INFO packet
          stringstream recieved_info;
          recieved_info << recv_buffer_.data();

          if( deserialize_pointcloud2_info_packet(recieved_info, Output_cloud, recv_data_length_int) == 1)
          {
            // Tell the server that the next packet should be data
            expected_message_stage = data;
            Confirm_reciept_str = "Got Info packet";

          } else {
            //throw error as we did not recieve complete infomation packet
            std_msgs::String msg;
            stringstream ss;
            ss << "Not Info packet -- data will being ignored";
            msg.data = ss.str();
            ROS_INFO("%s", msg.data.c_str());

            Confirm_reciept_str = "Expected info but didn't recieve info packet";
          }
        } else if (expected_message_stage == data) {
          // Unpack recieved data and check that it matches the length specified in previous info packet
          stringstream recieved_Data;
          stringstream rec_data;

          for(int cntr = 0; cntr < recv_data_length_int; cntr++)
          {
            rec_data << recv_buffer_[cntr];
          }

          recieved_Data << rec_data.str();
          std::string recieved_Data_str = recieved_Data.str();

          vector_deserialize(recieved_Data, Output_cloud.data);

          // publish the now complete output cloud
          point_cloud_publisher_.publish(Output_cloud);

          expected_message_stage = info;
          Confirm_reciept_str = "Got Data packet";
        }

    //Prep the confirmation output string
    boost::shared_ptr<std::string> message(new std::string(Confirm_reciept_str));

    // Send onfirmation that the packet was recieved
    socket_.async_send_to(boost::asio::buffer(*message), remote_endpoint_,
        boost::bind(&handle_send, message,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));

    // Listen for next packet
    start_receive();
  }
}



void handle_send(boost::shared_ptr<std::string> /*message*/,
      const boost::system::error_code& /*error*/,
      std::size_t /*bytes_transferred*/)
{

}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "PointCloud_UDP_Server");
  ros::NodeHandle node_;

  // Init all parameters - This will be replaced with ROS parameter server in final version
  Debug_Mode = false;
  output_topic = "points";
  Output_Buffer_Length = 100;

  // Initialiase variables
  expected_message_stage = info;

  // Advertise the output topic
  point_cloud_publisher_ = node_.advertise<pcl::PCLPointCloud2> (output_topic, Output_Buffer_Length, false);

  try
  {
    // Begin listening for data on specified port
    start_receive();
    io_context.run();
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
