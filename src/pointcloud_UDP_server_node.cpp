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
#include <pcl/filters/voxel_grid.h>

// Include my serialisation header for serialisation and de-serialisation functions
#include "serial_node/serialisation.hpp"


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

void Add_Recieved_Info(vector<string> split_info_stream);

void handle_send(boost::shared_ptr<std::string> /*message*/,
      const boost::system::error_code& /*error*/,
      std::size_t /*bytes_transferred*/);


std::string make_daytime_string()
{
  using namespace std; // For time_t, time and ctime;
  time_t now = time(0);
  return ctime(&now);
}

/* Function to listens on specified port for recieved point clouds */
void start_receive()
{
    socket_.async_receive_from(
        boost::asio::buffer(recv_buffer_), remote_endpoint_,
        boost::bind(&handle_receive,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
}



void Add_Recieved_Info(vector<string> split_info_stream)
{
  std::stringstream recv_height;
  std::stringstream recv_width;
  std::stringstream recv_point_step;
  std::stringstream recv_row_step;
  std::stringstream recv_is_dense;
  std::stringstream recv_data_length;

  vector<string> split_header_stream;
  boost::split(split_header_stream, split_info_stream[0], boost::is_any_of(";"));

  std::stringstream header_seq_re;
  std::stringstream header_stamp_re;
  std::stringstream header_frame_id_re;
  //
  header_seq_re << split_header_stream[0];
  header_stamp_re << split_header_stream[1];
  header_frame_id_re << split_header_stream[2];

  recv_height << split_info_stream[1];
  recv_width << split_info_stream[2];
  recv_point_step << split_info_stream[4];
  recv_row_step << split_info_stream[5];
  recv_is_dense << split_info_stream[6];
  recv_data_length << split_info_stream[7];

  // De-serialize the data length to be used when unpacking data packet
  recv_data_length_int = deserialize<int>(recv_data_length);

  // De-serialize the header
  Output_cloud.header.seq = deserialize<int>(header_seq_re);
  Output_cloud.header.stamp = deserialize<int>(header_stamp_re);
  Output_cloud.header.frame_id = header_frame_id_re.str();

  pcl::PCLPointCloud2 fields_cloud; // the reconstructed cloud to be output to the new ROS master
  // De-serialize the fields
  vector<string> point_types_vector_stream;
  boost::split(point_types_vector_stream, split_info_stream[3], boost::is_any_of(";"));

  for(int idx = 0; idx < (point_types_vector_stream.size() - 1); idx++)
  {
    vector<string> point_types_stuct_stream; // debug
    boost::split(point_types_stuct_stream, point_types_vector_stream[idx], boost::is_any_of(":"));

    if(point_types_stuct_stream.size() == 4)
    {
      pcl::PCLPointField temp_field;
      std::stringstream name_reconstruct;
      std::stringstream offset_reconstruct;
      std::stringstream datatype_reconstruct;
      std::stringstream count_reconstruct;

      name_reconstruct << point_types_stuct_stream[0];
      offset_reconstruct << point_types_stuct_stream[1];
      datatype_reconstruct << point_types_stuct_stream[2];
      count_reconstruct << point_types_stuct_stream[3];


      temp_field.name = name_reconstruct.str();
      temp_field.offset = deserialize<int>(offset_reconstruct);
      temp_field.datatype = deserialize<std::uint8_t>(datatype_reconstruct);
      temp_field.count = deserialize<int>(count_reconstruct);

      fields_cloud.fields.push_back(temp_field);

    } else {
      std_msgs::String msg;
      std::stringstream ss;
      ss << "ERROR: Size of point_types_vector_stream is: " << point_types_stuct_stream.size()  << endl;

      msg.data = ss.str();
      ROS_INFO("%s", msg.data.c_str());
    }
  }
  Output_cloud.fields = fields_cloud.fields;

  // De-serialize the simple varaiables
  Output_cloud.height = deserialize<int>(recv_height);
  Output_cloud.width = deserialize<int>(recv_width);
  Output_cloud.point_step = deserialize<int>(recv_point_step);
  Output_cloud.row_step = deserialize<int>(recv_row_step);

  Output_cloud.is_dense = deserialize<int>(recv_is_dense);

  // Set to zero as data not included in transmission packet
  Output_cloud.is_bigendian = 0;
}




/* Function is called when a point cloud is recieved */
void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if (!error)
    {
      //Prep an output string
      boost::shared_ptr<std::string> message(
          new std::string(make_daytime_string()));


      if(expected_message_stage == info) {
          // First check that the recieved msgs is the INFO packet
          stringstream recieved_info;
          recieved_info << recv_buffer_.data();
          vector<string> split_info_stream;
          boost::split(split_info_stream, recieved_info.str(),boost::is_any_of("\n"));

          if( ( split_info_stream.size() == 9 ) || ( split_info_stream.size() == 10 ) )
          {
            //Add the recieved info to the current ROS pointcloud
            Add_Recieved_Info(split_info_stream);

            // Tell the server that the next packet should be data
            expected_message_stage = data;
          } else {
            //throw error as we did not recieve complete infomation packet
            std_msgs::String msg;
            stringstream ss;
            ss << "Not Info packet -- data will being ignored";
            msg.data = ss.str();
            ROS_INFO("%s", msg.data.c_str());
          }

        }
        else if (expected_message_stage == data)
        {
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

          // output the cloud
          point_cloud_publisher_.publish(Output_cloud);

          // Tell the server that the next packet should be data
          expected_message_stage = info;
        }

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

  // Get all parameters for the client node
  node_.getParam("Debug_Mode", Debug_Mode);

  node_.getParam("output_topic", output_topic);
  node_.getParam("Output_Buffer_Length", Output_Buffer_Length);

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
