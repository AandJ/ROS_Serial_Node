// Include standard libraries
#include <iostream>

// Include ROS headers
#include "ros/ros.h"
#include "std_msgs/String.h"

// Include Boosts headers
#include <boost/array.hpp>
#include <boost/asio.hpp>

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
// using namespace Eigen;

typedef pcl::PointCloud<pcl::PointXYZRGB> RGB_PointCloud;


// Create subscriber and publisher for input and output of the Pointcloud data
ros::Subscriber scan_sub_;
void scanCallback(const std_msgs::String::ConstPtr& cloud_msg);

std_msgs::String msg;

// Server comm defintions
boost::asio::io_context io_context;
udp::resolver resolver(io_context);
udp::endpoint receiver_endpoint;
udp::socket socket_(io_context);

// Variables to store the parameters of the client
bool Debug_Mode;

std::string server_ip;
std::string server_port;
std::string input_topic;
int Input_Buffer_Length;

int Data_Output_Buffer_Size;
float LeafSize_X;
float LeafSize_Y;
float LeafSize_Z;

/* Function to recieve point clouds */
void scanCallback(const RGB_PointCloud::ConstPtr& cloud_msg){
  try
  {
/************* Input of data and conversion to POINTCLOUD2 data types *********/
      // Copy data to local variable
      pcl::PCLPointCloud2Ptr cloud_unfiltered(new pcl::PCLPointCloud2());
      pcl::toPCLPointCloud2(*cloud_msg, *cloud_unfiltered);



/*************************** Filter the input cloud ***************************/
      pcl::PCLPointCloud2 cloud_filtered;
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud (cloud_unfiltered);
      // sor.setLeafSize (0.01, 0.01, 0.01);
      sor.setLeafSize (LeafSize_X, LeafSize_Y, LeafSize_Z);
      sor.filter (cloud_filtered);   // Perform the voxel filtering


      // If debug is enabled output the size of the filtered cloud
      if(Debug_Mode)
      {
        std_msgs::String msg;
        // set data of ROS msg to the recieved string
        stringstream ss;
        ss << "Cloud size: ";// << cloud_filtered.data.size();
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
      }

/*********************** Serialize the filtered cloud  ************************/
      // Serialization streams for point cloud Data
      std::stringstream header;
      std::stringstream height;
      std::stringstream width;
      std::stringstream fields;
      std::stringstream is_bigendian;
      std::stringstream point_step;
      std::stringstream row_step;
      std::stringstream data;
      std::stringstream is_dense;

      // Serialize the HEADER
      std::stringstream header_seq;
      std::stringstream header_stamp;
      std::stringstream header_frame_id;

      serialize(header_seq, cloud_filtered.header.seq);
      serialize(header_stamp, cloud_filtered.header.stamp);
      header_frame_id << "map"; //cloud_filtered.header.frame_id;

      header << header_seq.str() << ';'  << header_stamp.str() << ';'  << header_frame_id.str();


      // Serialize the FIELDS
      for(int idx = 0; idx < cloud_filtered.fields.size(); idx ++)
      {
        pcl::PCLPointField temp_field = cloud_filtered.fields[idx];

        std::stringstream field_names;
        std::stringstream field_offset;
        std::stringstream field_datatype;
        std::stringstream field_count;

        field_names << temp_field.name;
        serialize(field_offset, temp_field.offset);
        serialize(field_datatype, temp_field.datatype);
        serialize(field_count, temp_field.count);

        fields << field_names.str() << ':' << field_offset.str() << ':' << field_datatype.str() << ':' << field_count.str() << ';' ;
      }

      // Serialize the the remaining fields using functions form serialisation.hpp
      serialize(height, cloud_filtered.height);
      serialize(width, cloud_filtered.width);
      serialize(is_bigendian, cloud_filtered.is_bigendian);
      serialize(point_step, cloud_filtered.point_step);
      serialize(row_step, cloud_filtered.row_step);
      vector_serialize(data, cloud_filtered.data);
      serialize(is_dense, cloud_filtered.is_dense);


/** Pack the Serialized data into the two info and data output stringstreams **/
      std::stringstream transmission_info;
      std::stringstream transmission_data;

      // First serialize the data so we can determine how much data is being sent
      transmission_data << data.str();
      std::string transmission_data_str = transmission_data.str();

      // Serialize the calculated data length to be included in info packet
      std::stringstream data_length;
      serialize(data_length, transmission_data_str.length());

      // Put transmission info in stringstream ready for output - BIGENDIAN removed as it caused the message to fail
      transmission_info << header.str() << endl << height.str() << endl << width.str() << endl << fields.str() << endl << point_step.str() << endl << row_step.str() << endl << is_dense.str() << endl << data_length.str() << endl;

      // Create STD string of output string stream
      std::string transmission_info_str = transmission_info.str();


      //Boost arrays for actual data to be output, initial Info header with data size, followed by large data packet of size specified in previous info packet
      boost::array<char, 200> send_transmission_info_buf;
      boost::array<char, 65000> send_transmission_data_buf;

      // Fill the arrays
      for(int cntr = 0; cntr < transmission_info_str.length(); cntr++)
      {
        send_transmission_info_buf[cntr] = transmission_info_str[cntr];
      }

      for(int cntr = 0; cntr < transmission_data_str.length(); cntr++)
      {
        send_transmission_data_buf[cntr] = transmission_data_str[cntr];
      }


      boost::asio::io_context io_context;

      udp::resolver resolver(io_context);
      udp::endpoint receiver_endpoint =
        *resolver.resolve(udp::v4(), server_ip, server_port).begin();

      udp::socket socket_(io_context);
      socket_.open(udp::v4());

      socket_.send_to(boost::asio::buffer(send_transmission_info_buf), receiver_endpoint);

      boost::array<char, 128> recv_buf;
      udp::endpoint sender_endpoint;
      size_t len = socket_.receive_from(
        boost::asio::buffer(recv_buf), sender_endpoint);



      socket_.send_to(boost::asio::buffer(send_transmission_data_buf), receiver_endpoint);

      boost::array<char, 128> recv_2_buf;
      udp::endpoint sender_endpoint_2;
      size_t len_2 = socket_.receive_from(
        boost::asio::buffer(recv_2_buf), sender_endpoint_2);

    }
    catch (std::exception& e)
    {
      std::cerr << e.what() << std::endl;
    }
}



int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pointcloud_UDP_client");
  ros::NodeHandle node_;

  // Get all parameters for the client node
  // node_.getParam("Debug_Mode", Debug_Mode);
  //
  // node_.getParam("/server_ip", server_ip);
  // node_.getParam("server_port", server_port);
  // node_.getParam("input_topic", input_topic);
  // node_.getParam("Input_Buffer_Length", Input_Buffer_Length);
  //
  // node_.getParam("Data_Output_Buffer_Size", Data_Output_Buffer_Size);
  // node_.getParam("LeafSize_X", LeafSize_X);
  // node_.getParam("LeafSize_Y", LeafSize_Y);
  // node_.getParam("LeafSize_Z", LeafSize_Z);

  Debug_Mode = false;

  server_ip = "192.168.1.203";
  server_port = "25000";
  input_topic = "/camera/depth/points";
  Input_Buffer_Length = 100;

  Data_Output_Buffer_Size = 65000;
  LeafSize_X = 0.15;
  LeafSize_Y = 0.15;
  LeafSize_Z = 0.15;


  receiver_endpoint = *resolver.resolve(udp::v4(), server_ip, server_port).begin();
  socket_.open(udp::v4());

  scan_sub_ = node_.subscribe<RGB_PointCloud> (input_topic, Input_Buffer_Length, &scanCallback);


  std_msgs::String msg;
  while (ros::ok())
  {

    try
    {
       ros::spinOnce();
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
