#include <string>
#include <iostream>
#include <cstdio>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <boost/asio.hpp>
#include <boost/array.hpp>

/**
 */


// Include Point Cloud Libraries (PCL)
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>  // Allows subscription to ROS topic with pcl::PointCloud<PointT> data types
#include <pcl/filters/voxel_grid.h>


// Debug Topic defintions to be replaced by ROS param get definition
#define PCL_Topic "/camera/depth_registered/points"

#define PI 3.141592654
#define Cloud_Buffer_Length 100

using namespace std;
using namespace ros;
using namespace Eigen;




// template<typename POD>
// std::ostream& custom_vector_serialize(std::ostream& os, std::vector<POD> const& v)
// {
//     // this only works on built in data types (PODs)
//     static_assert(std::is_trivial<POD>::value && std::is_standard_layout<POD>::value,
//         "Can only serialize POD types with this function");
//
//     auto size = v.size();
//     os.write(reinterpret_cast<char const*>(&size), sizeof(size));
//     os.write(reinterpret_cast<char const*>(v.data()), v.size() * sizeof(POD));
//     return os;
// }

// template<typename POD>
// std::istream& custom_vector_deserialize(std::istream& is, std::vector<POD>& v)
// {
//     static_assert(std::is_trivial<POD>::value && std::is_standard_layout<POD>::value,
//         "Can only deserialize POD types with this function");
//
//     decltype(v.size()) size;
//     is.read(reinterpret_cast<char*>(&size), sizeof(size));
//     v.resize(size);
//     is.read(reinterpret_cast<char*>(v.data()), v.size() * sizeof(POD));
//     return is;
// }



template<typename POD>
std::ostream& vector_serialize(std::ostream& os, std::vector<POD> const& v)
{
    // this only works on built in data types (PODs)
    static_assert(std::is_trivial<POD>::value && std::is_standard_layout<POD>::value,
        "Can only serialize POD types with this function");

    auto size = v.size();
    os.write(reinterpret_cast<char const*>(&size), sizeof(size));
    os.write(reinterpret_cast<char const*>(v.data()), v.size() * sizeof(POD));
    return os;
}

template<typename POD>
std::istream& vector_deserialize(std::istream& is, std::vector<POD>& v)
{
    static_assert(std::is_trivial<POD>::value && std::is_standard_layout<POD>::value,
        "Can only deserialize POD types with this function");

    decltype(v.size()) size;
    is.read(reinterpret_cast<char*>(&size), sizeof(size));
    v.resize(size);
    is.read(reinterpret_cast<char*>(v.data()), v.size() * sizeof(POD));
    return is;
}

template<typename T>
void serialize(std::ostream& os, const T& value)
{
    os << value;
}

template<typename T>
T deserialize(std::istream& is)
{
    T value;
    is >> value;
    return value;
}


typedef pcl::PointCloud<pcl::PointXYZRGB> RGB_PointCloud;

// Create PointCloud_Handler class to collect and combine the pointclouds for each direction
class PointCloud_Handler {
    public:
        PointCloud_Handler();

        void scanCallback(const RGB_PointCloud::ConstPtr& cloud_msg);
        Publisher point_cloud_publisher_;

    private:
        // Define node for ROS communication
        NodeHandle node_;

        // Create subscriber and publisher for input and output of the Pointcloud data
        Subscriber scan_sub_;

        // Debug Counter and msg
        int counter_;
        std_msgs::String msg;

};


// Class Constructor advertises output topic and subscribes to input topic
PointCloud_Handler::PointCloud_Handler(){
  counter_ = 0;
  point_cloud_publisher_ = node_.advertise<pcl::PCLPointCloud2> ("Test", Cloud_Buffer_Length, false);

  // Listen on the input topic for data
  scan_sub_ = node_.subscribe<RGB_PointCloud> (PCL_Topic, Cloud_Buffer_Length, &PointCloud_Handler::scanCallback, this);
}



/* Function to recieve right point clouds */
void PointCloud_Handler::scanCallback(const RGB_PointCloud::ConstPtr& cloud_msg){
/************* Input of data and conversion to POINTCLOUD2 data types *********/
  // Copy data to local variable
  pcl::PCLPointCloud2Ptr cloud_unfiltered(new pcl::PCLPointCloud2());
  pcl::toPCLPointCloud2(*cloud_msg, *cloud_unfiltered); // LINE 29!!


  /*************************** Filter the input cloud ***************************/
  pcl::PCLPointCloud2 cloud_filtered;
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_unfiltered);
  // sor.setLeafSize (0.01, 0.01, 0.01);
  sor.setLeafSize (0.08, 0.08, 0.08);
  sor.filter (cloud_filtered);   // Perform the voxel filtering


std_msgs::String msg;
// set data of ROS msg to the recieved string
stringstream ss;
ss << "Cloud size: " << cloud_filtered.data.size();
msg.data = ss.str();
ROS_INFO("%s", msg.data.c_str());

// Serialize
  // Serialization streams for point cloud Data
  std::stringstream header; // debug
  std::stringstream height;
  std::stringstream width;
  std::stringstream fields;
  std::stringstream is_bigendian;
  std::stringstream point_step;
  std::stringstream row_step;
  std::stringstream data;
  std::stringstream is_dense;

  std::stringstream transmission_info; // debug
  std::stringstream transmission_data;

  try
  {
    // Serialize the HEADER
    std::stringstream header_seq;
    std::stringstream header_stamp;
    std::stringstream header_frame_id;

    serialize(header_seq, cloud_filtered.header.seq);
    serialize(header_stamp, cloud_filtered.header.stamp);
    header_frame_id << cloud_filtered.header.frame_id;

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

    // Serialize the easy ones
    serialize(height, cloud_filtered.height);
    serialize(width, cloud_filtered.width);
    serialize(is_bigendian, cloud_filtered.is_bigendian);
    serialize(point_step, cloud_filtered.point_step);
    serialize(row_step, cloud_filtered.row_step);
    vector_serialize(data, cloud_filtered.data);
    serialize(is_dense, cloud_filtered.is_dense);

    // Put transmission data in stringstream ready for output
    transmission_data << data.str();
    std::string transmission_data_str = transmission_data.str();

    // Serialize the length of the data to be included in info packet
    std::stringstream data_length;
    serialize(data_length, transmission_data_str.length());

    // Put transmission info in stringstream ready for output - BIGENDIAN removed as it caused the message to fail
    transmission_info << header.str() << endl << height.str() << endl << width.str() << endl << fields.str() << endl << point_step.str() << endl << row_step.str() << endl << is_dense.str() << endl << data_length.str() << endl;

    // Create STD string of output string stream
    std::string transmission_info_str = transmission_info.str();


    //Boost arrays for actual data to be output, initial Info header with data size, followed by large data packet of size specified in previous info packet
    boost::array<char, 200> send_transmission_info_buf;
    boost::array<char, 1000000> send_transmission_data_buf;

    // Fill the arrays
    for(int cntr = 0; cntr < transmission_info_str.length(); cntr++)
    {
      send_transmission_info_buf[cntr] = transmission_info_str[cntr];
    }

    for(int cntr = 0; cntr < transmission_data_str.length(); cntr++)
    {
      send_transmission_data_buf[cntr] = transmission_data_str[cntr];
    }

    //Output the Boost arrays here
    std_msgs::String msg;
    // // set data of ROS msg to the recieved string
    // stringstream ss;
    // ss << "transmission_info_str: " << transmission_info_str.length();
    // msg.data = ss.str();
    // ROS_INFO("%s", msg.data.c_str());
    //
    // std_msgs::String msg2;
    // // set data of ROS msg to the recieved string
    // stringstream ss2;
    // ss2 << "transmission_data_str: " << transmission_data_str.length();
    // msg2.data = ss2.str();
    // ROS_INFO("%s", msg2.data.c_str());






/************************************* Reciever side ******************************************************/


    std::stringstream Recieved_Info;
    std::stringstream Recieved_Data;

    // Recieve the info and data packets
    Recieved_Info << send_transmission_info_buf.data();

    vector<string> split_info_stream;
    boost::split(split_info_stream, Recieved_Info.str(),boost::is_any_of("\n"));

    // set data of ROS msg to the recieved string
    stringstream ss;
    ss << "Got Data: " << split_info_stream.size();
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());


    std::stringstream recv_height;
    std::stringstream recv_width;
    std::stringstream recv_point_step;
    std::stringstream recv_row_step;
    std::stringstream recv_data;
    std::stringstream recv_is_dense;
    std::stringstream recv_data_length;

    // recv_header << split_full_stream[0];
    recv_height << split_info_stream[1];
    recv_width << split_info_stream[2];
    // recv_is_bigendian << split_info_stream[4];
    recv_point_step << split_info_stream[4];
    recv_row_step << split_info_stream[5];
    recv_is_dense << split_info_stream[6];
    recv_data_length << split_info_stream[7];

    int recv_data_length_int = deserialize<int>(recv_data_length);
    for(int cntr = 0; cntr < recv_data_length_int; cntr++)
    {
      Recieved_Data << send_transmission_data_buf[cntr];
    }

    recv_data << Recieved_Data.str();





    // Unpack the info and data packets into PCL pointcloud
    pcl::PCLPointCloud2 new_cloud; // the reconstructed cloud to be output to the new ROS master

    vector<string> split_header_stream;
    boost::split(split_header_stream, split_info_stream[0], boost::is_any_of(";"));

    std::stringstream header_seq_re;
    std::stringstream header_stamp_re;
    std::stringstream header_frame_id_re;
    //
    header_seq_re << split_header_stream[0];
    header_stamp_re << split_header_stream[1];
    header_frame_id_re << split_header_stream[2];


    new_cloud.header.seq = deserialize<int>(header_seq_re);
    new_cloud.header.stamp = deserialize<int>(header_stamp_re);
    new_cloud.header.frame_id = header_frame_id_re.str();


    // de-serialize the fields
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

        new_cloud.fields.push_back(temp_field);

      } else {
        std::stringstream ss;
        ss << "ERROR: Size of point_types_vector_stream is: " << point_types_stuct_stream.size()  << endl;

        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
      }
    }

    new_cloud.height = deserialize<int>(recv_height);
    new_cloud.width = deserialize<int>(recv_width);

    new_cloud.is_bigendian = 0; // Set to zero as data not included in transmission packet

    new_cloud.point_step = deserialize<int>(recv_point_step);
    new_cloud.row_step = deserialize<int>(recv_row_step);
    vector_deserialize(recv_data, new_cloud.data);
    new_cloud.is_dense = deserialize<int>(recv_is_dense);

    point_cloud_publisher_.publish(new_cloud);
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return;
}





int main(int argc, char **argv)
{

  ros::init(argc, argv, "test");
  PointCloud_Handler PCL_Handler;

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    spin();
  }

  return 0;
}
