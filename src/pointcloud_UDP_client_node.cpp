#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

// Include Point Cloud Libraries (PCL)
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>  // Allows subscription to ROS topic with pcl::PointCloud<PointT> data types
#include <pcl/filters/voxel_grid.h>


using boost::asio::ip::udp;
using namespace std;
using namespace ros;
using namespace Eigen;

#define PCL_Topic "/camera/depth_registered/points"

#define PI 3.141592654
#define Cloud_Buffer_Length 100

typedef pcl::PointCloud<pcl::PointXYZRGB> RGB_PointCloud;





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












// Create PointCloud_Handler class to collect and combine the pointclouds for each direction
class PointCloud_UDP_Transmiter {
    public:
        PointCloud_UDP_Transmiter();

        void scanCallback(const RGB_PointCloud::ConstPtr& cloud_msg);
    private:
        // Define node for ROS communication
        NodeHandle node_;

        // Create subscriber and publisher for input and output of the Pointcloud data
        Subscriber scan_sub_;
};

// Class Constructor advertises output topic and subscribes to input topic
PointCloud_UDP_Transmiter::PointCloud_UDP_Transmiter(){
  // Listen on the input topic for data
  scan_sub_ = node_.subscribe<RGB_PointCloud> (PCL_Topic, Cloud_Buffer_Length, &PointCloud_UDP_Transmiter::scanCallback, this);
}


/* Function to recieve right point clouds */
void PointCloud_UDP_Transmiter::scanCallback(const RGB_PointCloud::ConstPtr& cloud_msg){
  /************* Input of data and conversion to POINTCLOUD2 data types *********/
    // Copy data to local variable
    pcl::PCLPointCloud2Ptr cloud_unfiltered(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*cloud_msg, *cloud_unfiltered); // LINE 29!!


    /*************************** Filter the input cloud ***************************/
    pcl::PCLPointCloud2 cloud_filtered;
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud_unfiltered);
    // sor.setLeafSize (0.01, 0.01, 0.01);
    sor.setLeafSize (0.15, 0.15, 0.15);
    sor.filter (cloud_filtered);   // Perform the voxel filtering

    std_msgs::String msg;
    // set data of ROS msg to the recieved string
    stringstream ss;
    ss << "Cloud size: " << cloud_filtered.data.size();
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());

  // Serialize
    // Debug cloud to check serialization
    pcl::PCLPointCloud2 new_cloud;


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
        *resolver.resolve(udp::v4(), "192.168.1.203", "25000").begin();

      udp::socket socket(io_context);
      socket.open(udp::v4());



      socket.send_to(boost::asio::buffer(send_transmission_info_buf), receiver_endpoint);

      boost::array<char, 128> recv_buf;
      udp::endpoint sender_endpoint;
      size_t len = socket.receive_from(
        boost::asio::buffer(recv_buf), sender_endpoint);



      socket.send_to(boost::asio::buffer(send_transmission_data_buf), receiver_endpoint);

      boost::array<char, 128> recv_2_buf;
      udp::endpoint sender_endpoint_2;
      size_t len_2 = socket.receive_from(
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

  PointCloud_UDP_Transmiter UDP_Transmiter;

  // ros::Rate loop_rate(10);

  while (ros::ok())
  {

    try
    {
      // std_msgs::String msg;

      // if (argc != 2)
      // {
      //   std::cerr << "Usage: client <host>" << std::endl;
      //   return 1;
      // }

      // boost::asio::io_context io_context;
      //
      // udp::resolver resolver(io_context);
      // udp::endpoint receiver_endpoint =
      //   *resolver.resolve(udp::v4(), "192.168.1.203", "25000").begin();
      //
      // udp::socket socket(io_context);
      // socket.open(udp::v4());
      //
      // boost::array<char, 2> send_buf  = {{ '1','1' }};
      //
      // socket.send_to(boost::asio::buffer(send_buf), receiver_endpoint);
      //
      // boost::array<char, 128> recv_buf;
      // udp::endpoint sender_endpoint;
      // size_t len = socket.receive_from(
      //     boost::asio::buffer(recv_buf), sender_endpoint);
      //
      // std::cout.write(recv_buf.data(), len);

      // std::string S(recv_buf.data());
      // set data of ROS msg to the recieved string
      // msg.data = "Point 2";
      // ROS_INFO("%s", msg.data.c_str());
      spin();

    }
    catch (std::exception& e)
    {
      std::cerr << e.what() << std::endl;
    }

  }


  return 0;
}
