//----------------------------------------------------------------------
#include <string>
#include "std_msgs/String.h"
#include "ros/ros.h"


#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_ros/transforms.h>
// #include <pcl_ros/point_cloud.h>  // Allows subscription to ROS topic with pcl::PointCloud<PointT> data types
// #include <pcl/filters/voxel_grid.h>

// #define PCL_Topic "/camera/depth_registered/points"
//
//
// using boost::asio::ip::tcp;
// using namespace ros;
//
// typedef pcl::PointCloud<pcl::PointXYZRGB> RGB_PointCloud;
//
// // Create PointCloud_Handler class to collect and combine the pointclouds for each direction
// class PointCloud_Transmiter {
//     public:
//         PointCloud_Transmiter();
//
//         void scanCallback(const RGB_PointCloud::ConstPtr& cloud_msg);
//     private:
//         // Define node for ROS communication
//         NodeHandle node_;
//
//         // Create subscriber and publisher for input and output of the Pointcloud data
//         Subscriber scan_sub_;
// };
//
// // Class Constructor advertises output topic and subscribes to input topic
// PointCloud_Handler::PointCloud_Handler(){
//   // Listen on the input topic for data
//   scan_sub_ = node_.subscribe<RGB_PointCloud> (PCL_Topic, Cloud_Buffer_Length, &PointCloud_Handler::scanCallback, this);
// }
//
//
// /* Function to recieve right point clouds */
// void PointCloud_Handler::scanCallback(const RGB_PointCloud::ConstPtr& cloud_msg){
//   boost::asio::io_context io_context;
//
//   tcp::resolver resolver(io_context);
//   tcp::resolver::results_type endpoints =
//     resolver.resolve("192.168.1.203", "25000");
//
//   tcp::socket socket(io_context);
//   boost::asio::connect(socket, endpoints);
//
//   for (;;)
//   {
//     boost::array<char, 128> buf;
//     boost::system::error_code error;
//
//     size_t len = socket.read_some(boost::asio::buffer(buf), error);
//
//     if (error == boost::asio::error::eof)
//       break; // Connection closed cleanly by peer.
//     else if (error)
//       throw boost::system::system_error(error); // Some other error.
//
//     // std::cout.write(buf.data(), len);
//
//     char array[len] = buf.data();
//     string msg = "";
//     for (int i = 0; i < len; i++) {
//         msg = msg + array[i];
//     }
//     ROS_INFO("%s", msg.data.c_str());
//   }
//
// }
//
//
//
//
//
//
//
int main(int argc, char* argv[])
{
//   ros::init(argc, argv, "pointcloud_TCP_client");
//   ros::NodeHandle node_;
//
//   PointCloud_Transmiter PCL_Transmiter;
//
//   spin();
//
//   // try
//   // {
//   //
//   //
//   // }
//   // catch (std::exception& e)
//   // {
//   //   std::cerr << e.what() << std::endl;
//   // }
//
  return 0;
}
