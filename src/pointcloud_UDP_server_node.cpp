#include "ros/ros.h"
#include "std_msgs/String.h"

#include <ctime>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>


// Include Point Cloud Libraries (PCL)
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>  // Allows subscription to ROS topic with pcl::PointCloud<PointT> data types
#include <pcl/filters/voxel_grid.h>


// Debug Topic defintions to be replaced by ROS param get definition
#define PCL_Topic "/points"

#define PI 3.141592654
#define Cloud_Buffer_Length 100


using boost::asio::ip::udp;
using namespace std;
using namespace ros;
using namespace Eigen;

Publisher point_cloud_publisher_;




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










std::string make_daytime_string()
{
  using namespace std; // For time_t, time and ctime;
  time_t now = time(0);
  return ctime(&now);
}

class udp_server
{
public:
  udp_server(boost::asio::io_context& io_context)
    : socket_(io_context, udp::endpoint(udp::v4(), 25000))
  {
    debug = 0;

    start_receive();
  }

private:
  void start_receive()
  {
    socket_.async_receive_from(
        boost::asio::buffer(recv_buffer_), remote_endpoint_,
        boost::bind(&udp_server::handle_receive, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
  }

  void handle_receive(const boost::system::error_code& error,
      std::size_t bytes_transferred)
  {
    if (!error)
    {
      //Prep an output string
      boost::shared_ptr<std::string> message(
          new std::string(make_daytime_string()));


      if(debug == 0) // The Infomation packet
      {
        // Empty the existing cloud
        // Output_cloud.clear();

        stringstream recieved_info;
        recieved_info << recv_buffer_.data();

        vector<string> split_info_stream;
        boost::split(split_info_stream, recieved_info.str(),boost::is_any_of("\n"));


                // std_msgs::String msg;
                // // set data of ROS msg to the recieved string
                // stringstream ss;
                // ss << "packet - " << recieved_info.str();
                // msg.data = ss.str();
                // ROS_INFO("%s", msg.data.c_str());
                //
                // std_msgs::String msg2;
                // // set data of ROS msg to the recieved string
                // stringstream ss2;
                // ss2 << "packet items - " << split_info_stream.size();
                // msg2.data = ss2.str();
                // ROS_INFO("%s", msg2.data.c_str());
                //
                // std_msgs::String msg3;
                // // set data of ROS msg to the recieved string
                // stringstream ss3;
                // ss3 << "transfered - " << bytes_transferred;
                // msg3.data = ss3.str();
                // ROS_INFO("%s", msg3.data.c_str());


        if( ( split_info_stream.size() == 9 ) || ( split_info_stream.size() == 10 ) )
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

          debug = 1;
        } else {
          //throw error as we did not recieve complete infomation packet
          std_msgs::String msg;
          // set data of ROS msg to the recieved string
          stringstream ss;
          ss << "Not Info packet";
          msg.data = ss.str();
          ROS_INFO("%s", msg.data.c_str());
        }

      } else {  // The data packet
        stringstream recieved_Data;
        stringstream rec_data;

        for(int cntr = 0; cntr < recv_data_length_int; cntr++)
        {
          rec_data << recv_buffer_[cntr];
        }

        recieved_Data << rec_data.str();
        std::string recieved_Data_str = recieved_Data.str();

        // std_msgs::String msg;
        // // set data of ROS msg to the recieved string
        // stringstream ss;
        // ss << "data str - " << recieved_Data.str();
        // msg.data = ss.str();
        // ROS_INFO("%s", msg.data.c_str());
        //
        // std_msgs::String msg2;
        // // set data of ROS msg to the recieved string
        // stringstream ss2;
        // ss2 << "data length - " << recieved_Data_str.length();
        // msg2.data = ss2.str();
        // ROS_INFO("%s", msg2.data.c_str());
        //
        // std_msgs::String msg3;
        // // set data of ROS msg to the recieved string
        // stringstream ss3;
        // ss3 << "recv_data_length_int - " << recv_data_length_int;
        // msg3.data = ss3.str();
        // ROS_INFO("%s", msg3.data.c_str());

        vector_deserialize(recieved_Data, Output_cloud.data);

        // and output the cloud
        point_cloud_publisher_.publish(Output_cloud);

        debug = 0;
      }


      // Send current date time to confirm data recieved
      socket_.async_send_to(boost::asio::buffer(*message), remote_endpoint_,
          boost::bind(&udp_server::handle_send, this, message,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));

      start_receive();
    }
  }

  void handle_send(boost::shared_ptr<std::string> /*message*/,
      const boost::system::error_code& /*error*/,
      std::size_t /*bytes_transferred*/)
  {
  }

  udp::socket socket_;
  udp::endpoint remote_endpoint_;
  boost::array<char, 65000> recv_buffer_;

  pcl::PCLPointCloud2 Output_cloud; // the reconstructed cloud to be output to the new ROS master

  int debug;
  int recv_data_length_int;

  std_msgs::String msg; //debug rosout msg

};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_UDP_server");
  ros::NodeHandle node_;

  point_cloud_publisher_ = node_.advertise<pcl::PCLPointCloud2> (PCL_Topic, Cloud_Buffer_Length, false);

  try
  {
    boost::asio::io_context io_context;
    udp_server server(io_context);
    io_context.run();
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
