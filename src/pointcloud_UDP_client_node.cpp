// Include standard libraries
#include <iostream>

// Include ROS headers
#include "ros/ros.h"
#include "std_msgs/String.h"


// Include Point Cloud Libraries (PCL)
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>  // Allows subscription to ROS topic with pcl::PointCloud<PointT> data types


#include "serial_node/pointcloud_processing.hpp"
#include "serial_node/boost_comms.hpp"


using namespace std;
using namespace ros;


typedef pcl::PointCloud<pcl::PointXYZRGB> XYZRGB_PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> XYZ_PointCloud;


// Create subscriber and publisher for input and output of the Pointcloud data
ros::Subscriber scan_sub_;
void scanCallback(const std_msgs::String::ConstPtr& cloud_msg);

// Debug ROS publisher to output the filter pointcloud
Publisher debug_point_cloud_publisher_;


// Variables to store the parameters of the client
bool Debug_Mode;

std::string server_ip;
std::string server_port;
std::string input_topic;
int Input_Buffer_Length;

bool colour_mode;

int Data_Output_Buffer_Size;
float LeafSize_X;
float LeafSize_Y;
float LeafSize_Z;


// Scan callback is called when a scan is recieved, it filter, serialises and then outputs the scan via Boost UDP
void scanCallback(const XYZRGB_PointCloud::ConstPtr& cloud_msg){
  try
  {
      // Copy data to local variable
      pcl::PCLPointCloud2Ptr cloud_unfiltered(new pcl::PCLPointCloud2());
      pcl::toPCLPointCloud2(*cloud_msg, *cloud_unfiltered);

      // Filter the input cloud to below 65000 bytes in sizer
      pcl::PCLPointCloud2 cloud_filtered = Dynamic_Pointcloud_Filter(cloud_unfiltered);

      // If debug is enabled output the the filtered cloud to the debug topic
      if(Debug_Mode)
      {
          debug_point_cloud_publisher_.publish(cloud_filtered);
      }

      Serialised_pointcloud2_Struct Serialised_pointcloud2;

      // Check the filtered cloud does not exceed allowed size
      if(cloud_filtered.data.size() < 65000)
      {
        // Serialise the filter cloud to enable output via Boost
        // Serialised_pointcloud2_Struct Serialised_pointcloud2;
        Serialised_pointcloud2 = Serialise_Pointcloud2(cloud_filtered);

      } else {
        // Print error and ignore the recieved pointcloud
        std_msgs::String msg;
        stringstream ss;
        ss << "Error: Cloud size too large - " << cloud_filtered.data.size();
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
      }

      // Output via boost
      UDP_Transmit_Pointcloud2(Serialised_pointcloud2, server_ip, server_port);

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

  // Init all parameters - This will be replaced with ROS parameter server in final version
  Debug_Mode = true;

  server_ip = "192.168.1.203";
  server_port = "25000";
  // input_topic = "/camera/depth/points";
  input_topic = "/camera/depth_registered/points";
  Input_Buffer_Length = 1;

  colour_mode = true;

  Data_Output_Buffer_Size = 65000; // Not implemented
  LeafSize_X = 0.15;
  LeafSize_Y = 0.15;
  LeafSize_Z = 0.15;


  if(Debug_Mode)
  {
    debug_point_cloud_publisher_ = node_.advertise<pcl::PCLPointCloud2> ("/Debug_PointCloud", 1, false);
  }


  scan_sub_ = node_.subscribe<XYZRGB_PointCloud> (input_topic, Input_Buffer_Length, &scanCallback);


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
        std_msgs::String msg;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        return 1;
    }
  }

  return 0;
}
