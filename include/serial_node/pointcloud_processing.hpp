// Include Point Cloud Libraries (PCL)
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>  // Allows subscription to ROS topic with pcl::PointCloud<PointT> data types
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>

// Include my serialisation header for serialisation and de-serialisation functions
#include "serial_node/serialisation.hpp"


using namespace std;


// Struct for holding serialised pointcloud2
typedef struct{
  std::string info;
  std::string data;
}Serialised_pointcloud2_Struct;



/*** Filter the recieved cloud down to a max size of 65000 bytes (Max for boost array)***/
pcl::PCLPointCloud2 Dynamic_Pointcloud_Filter(pcl::PCLPointCloud2Ptr input_cloud)
{
        // Create the object to hold the output cloud
        pcl::PCLPointCloud2 filtered_cloud;


        pcl::PCLPointCloud2 init_filtered_cloud;
        pcl::VoxelGrid<pcl::PCLPointCloud2> init_sor;
        init_sor.setInputCloud (input_cloud);
        init_sor.setLeafSize (0.01, 0.01, 0.01);
        init_sor.filter (init_filtered_cloud);   // Perform the voxel filtering

        //if scan is too big re filter with leaf size based from size of scan, bigger scan bigger leaf size
        int input_size = init_filtered_cloud.data.size();
        if(input_size < 65000)
        { // If cloud is already smaller than max size then save as the output cloud
          filtered_cloud = init_filtered_cloud;
        } else {  // Else calculate filter leaf values based on the cloud size and re-filter
          int maxInRange = 9830400;
          int minInRange = 0;
          float maxOutRange = 0.45;
          float minOutRange = 0.03;

          float mapped_filter_vals = minOutRange + (input_size - minInRange) * (maxOutRange - minOutRange) / (maxInRange - minInRange);

          pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
          sor.setInputCloud (input_cloud);
          sor.setLeafSize (mapped_filter_vals, mapped_filter_vals, mapped_filter_vals);
          sor.filter (filtered_cloud);
        }

        return filtered_cloud;
}




Serialised_pointcloud2_Struct Serialise_Pointcloud2(pcl::PCLPointCloud2 input_cloud)
{
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

  serialize(header_seq, input_cloud.header.seq);
  serialize(header_stamp, input_cloud.header.stamp);
  // Debug set the frame id to map so it can be visualised without publising a TF tree
  header_frame_id << "map"; //input_cloud.header.frame_id;

  header << header_seq.str() << ';'  << header_stamp.str() << ';'  << header_frame_id.str();

  // Serialize the FIELDS
  for(int idx = 0; idx < input_cloud.fields.size(); idx ++)
  {
    pcl::PCLPointField temp_field = input_cloud.fields[idx];

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
  serialize(height, input_cloud.height);
  serialize(width, input_cloud.width);
  serialize(is_bigendian, input_cloud.is_bigendian);
  serialize(point_step, input_cloud.point_step);
  serialize(row_step, input_cloud.row_step);
  vector_serialize(data, input_cloud.data);
  serialize(is_dense, input_cloud.is_dense);

  Serialised_pointcloud2_Struct Output_Serialised_pointcloud2;

  /** Pack the Serialized data into the two info and data output stringstreams **/
  std::stringstream output_info;
  std::stringstream output_data;

  // First serialize the data so we can determine how much data is being sent
  output_data << data.str();
  Output_Serialised_pointcloud2.data = output_data.str();

  // Serialize the calculated data length to be included in info packet
  std::stringstream data_length;
  serialize(data_length, Output_Serialised_pointcloud2.data.length());

  // Put transmission info in stringstream ready for output - BIGENDIAN removed as it caused the message to fail
  output_info << header.str() << endl << height.str() << endl << width.str() << endl << fields.str() << endl << point_step.str() << endl << row_step.str() << endl << is_dense.str() << endl << data_length.str() << endl;

  // Create STD string of output string stream
  Output_Serialised_pointcloud2.info = output_info.str();

  return Output_Serialised_pointcloud2;
}







int deserialize_pointcloud2_info_packet(stringstream& serialised_info, pcl::PCLPointCloud2& cloud, int& expected_data_length)
{
  vector<string> split_info_stream;
  boost::split(split_info_stream, serialised_info.str(),boost::is_any_of("\n"));

  // Check recieve data matches info packet format
  if( ( split_info_stream.size() != 9 ) && ( split_info_stream.size() != 10 ) )
  {
    //do error thing
    return 2; // return 2 to show recieved data does not match info packet
  } else {
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
    expected_data_length = deserialize<int>(recv_data_length);

    // De-serialize the header
    cloud.header.seq = deserialize<int>(header_seq_re);
    cloud.header.stamp = deserialize<int>(header_stamp_re);
    cloud.header.frame_id = header_frame_id_re.str();

    pcl::PCLPointCloud2 temp_fields_cloud;

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

        temp_fields_cloud.fields.push_back(temp_field);

      } else {
        return 3; // return 3 to show recieved info packet is corrupted
      }
    }
    cloud.fields = temp_fields_cloud.fields;

    // De-serialize the simple varaiables
    cloud.height = deserialize<int>(recv_height);
    cloud.width = deserialize<int>(recv_width);
    cloud.point_step = deserialize<int>(recv_point_step);
    cloud.row_step = deserialize<int>(recv_row_step);

    cloud.is_dense = deserialize<int>(recv_is_dense);

    // Set to zero as data not included in transmission packet
    cloud.is_bigendian = 0;

    return 1; // return 1 to show info packet was correctly serialised
  }
}
