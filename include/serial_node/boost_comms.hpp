// Include Boosts headers
#include <boost/array.hpp>
#include <boost/asio.hpp>


using boost::asio::ip::udp;


void UDP_Transmit_Pointcloud2(Serialised_pointcloud2_Struct Serialised_pointcloud2, std::string server_ip, std::string server_port)
{
  //Boost arrays for actual data to be output, initial Info header with data size, followed by large data packet of size specified in previous info packet
  boost::array<char, 200> send_transmission_info_buf;
  boost::array<char, 65000> send_transmission_data_buf;

  // Convert the recieved Serialised_pointcloud2_Struct into Boost arrays
  for(int cntr = 0; cntr < Serialised_pointcloud2.info.length(); cntr++)
  {
    send_transmission_info_buf[cntr] = Serialised_pointcloud2.info[cntr];
  }

  for(int cntr = 0; cntr < Serialised_pointcloud2.data.length(); cntr++)
  {
    send_transmission_data_buf[cntr] = Serialised_pointcloud2.data[cntr];
  }


  boost::asio::io_context io_context;

  udp::resolver resolver(io_context);
  udp::endpoint receiver_endpoint =
    *resolver.resolve(udp::v4(), server_ip, server_port).begin();

  udp::socket socket_(io_context);
  socket_.open(udp::v4());


  // if(Debug_Mode)
  // {
  //   ROS_INFO("sending info");
  // }

  std::string returned_msg_check_str = "init";

  do{ // Send info packet until server responds that it has it
    socket_.send_to(boost::asio::buffer(send_transmission_info_buf), receiver_endpoint);

    boost::array<char, 128> recv_buf;
    udp::endpoint sender_endpoint;
    size_t len = socket_.receive_from(
      boost::asio::buffer(recv_buf), sender_endpoint);

    // if(Debug_Mode)
    // {
    //   std_msgs::String msg1;
    //   // ERROR condition for access to the serial port
    //   std::stringstream ss1;
    //   ss1 << recv_buf.data();
    //   msg1.data = ss1.str();
    //   ROS_INFO("%s", msg1.data.c_str());
    // }

    returned_msg_check_str = recv_buf.data();
  } while(returned_msg_check_str != "Got Info packet");

  // Now the server has the info, send the data
  socket_.send_to(boost::asio::buffer(send_transmission_data_buf), receiver_endpoint);

  boost::array<char, 128> recv_2_buf;
  udp::endpoint sender_endpoint_2;
  size_t len_2 = socket_.receive_from(
    boost::asio::buffer(recv_2_buf), sender_endpoint_2);

  returned_msg_check_str = recv_2_buf.data();

  if(returned_msg_check_str == "Got Data packet")
  {
    // if(Debug_Mode)
    // {
    //   std_msgs::String msg2;
    //   // ERROR condition for access to the serial port
    //   std::stringstream ss2;
    //   ss2 << recv_2_buf.data();
    //   msg2.data = ss2.str();
    //   ROS_INFO("%s", msg2.data.c_str());
    // }
  } else {
    //throw error as server did not recieve data packet
    std_msgs::String msg;
    stringstream ss;
    ss << "data send failed, scan will be ignored";
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
  }
}
