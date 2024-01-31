# ROS_Serial_Node

This ROS Package will provide a variety of ROS wrappers for standard forms of communication such as USB, TCP, & UDP to enable communication using ROS topics to a target device.  

# The wrappers  

## serial_node
This program enables ROS to commincate via USB, providing an input topic for data to be sent to the target device, and an output topic where data recieved from the target device is published.  

This program is launched using the serial_node.launch launch file which specified the name of the input and output topic aswell as the address of the target device and Baud Rate.  

## int_serial_node
Provides the same functionallity of the as the serial node program, however transmits 32bit integers instead of strings of undefined length. These 32bit integers are broken up into 5 chars, the first four containing 7 bits and the last containing 4 bits and a checksum. This provides more efficient transfer of 32bit ints than using the standard serial_node and serialising the data before transmission.  

## pointcloud_TCP_client/server_node
Currently not functional

## pointcloud_UDP_client/server_node
Provides the ability to transmit raw PCL::Pointcloud2 data (Including XYZ and XYZRGB variants) via the UDP protocol from a client to a server specified by its IP. This program has two seperate launch files, the server should be launched first, followed by the client on the device connected to the depth camera.

Currently parameter configuration through the launch files does not work and so these must be modified within the Cpp themselves. (This will be fixed)
Current testing shows that a max of 4 frames can be transmitted per second. This will be increased to atleast 8 FPS for XYZ varients in the final release.  
