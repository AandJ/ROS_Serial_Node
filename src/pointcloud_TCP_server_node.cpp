// #include "pointcloud_TCP_server_node.hpp"

//
// _________.cpp
// ~~~~~~~~~~
// Taken from the Boost example Daytime.3 - An asynchronous TCP daytime server
//

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <ctime>
#include <iostream>
#include <string>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;


std::string make_daytime_string()
{
  using namespace std; // For time_t, time and ctime;
  time_t now = time(0);
  return ctime(&now);
}

class tcp_connection : public boost::enable_shared_from_this<tcp_connection>
{
public:
  typedef boost::shared_ptr<tcp_connection> pointer;

  static pointer create(boost::asio::io_context& io_context)
  {
    return pointer(new tcp_connection(io_context));
  }

  tcp::socket& socket()
  {
    return socket_;
  }

  void start()
  {
    message_ = make_daytime_string();

    std_msgs::String msg;
    // set data of ROS msg to the recieved string
    msg.data = message_;
    ROS_INFO("%s", msg.data.c_str());

    boost::asio::async_write(socket_, boost::asio::buffer(message_),
        boost::bind(&tcp_connection::handle_write, shared_from_this(),
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
  }

private:
  tcp_connection(boost::asio::io_context& io_context)
    : socket_(io_context)
  {
  }

  void handle_write(const boost::system::error_code& /*error*/,
      size_t /*bytes_transferred*/)
  {
  }

  tcp::socket socket_;
  std::string message_;
};

class tcp_server
{
public:
  tcp_server(boost::asio::io_context& io_context)
    : io_context_(io_context),
      acceptor_(io_context, tcp::endpoint(tcp::v4(), 25000))
  {
    start_accept();
  }

private:
  void start_accept()
  {
    tcp_connection::pointer new_connection = tcp_connection::create(io_context_);

    acceptor_.async_accept(new_connection->socket(), boost::bind(&tcp_server::handle_accept, this, new_connection, boost::asio::placeholders::error));
  }

  void handle_accept(tcp_connection::pointer new_connection,
      const boost::system::error_code& error)
  {
    if (!error)
    {
      new_connection->start();
    }

    start_accept();
  }

  boost::asio::io_context& io_context_;
  tcp::acceptor acceptor_;
};






//----------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_TCP_server");
  ros::NodeHandle node_;

  // std_msgs::String msg;
  // ros::Rate loop_rate(10);

  // Loop only
  while (ros::ok())
  {
    try
    {
      boost::asio::io_context io_context;
      tcp_server server(io_context);
      io_context.run();
    }
    catch (std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      ROS_INFO("I crashed");

    }

    return 0;
  }
}
