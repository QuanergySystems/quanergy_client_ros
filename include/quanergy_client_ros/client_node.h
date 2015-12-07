/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#ifndef QUANERGY_CLIENT_ROS_CLIENT_NODE_H
#define QUANERGY_CLIENT_ROS_CLIENT_NODE_H

#include <string>
#include <cstdint>


#include <quanergy_client_ros/simple_publisher.h>

struct ClientNode
{

  ClientNode(int argc, char** argv);

  void publish();

private:

  void loadSettings(int argc, char ** argv);
  void parseArgs(int argc, char ** argv);

private:

  // Defaults, overridded by settings file, which is in turn
  // overridden by command-line options.

  std::string settings_file;

  // Distance filter

  float min = 0.5f;
  float max = 500.f;

  // Ring filter
  // Only set by config file
  // only relevant for M8

  float default_ring_range = 0.f;
  std::uint16_t default_ring_intensity = 0;

  float ring_range[quanergy::client::M8_NUM_LASERS];
  std::uint16_t ring_intensity[quanergy::client::M8_NUM_LASERS];
  
  // Client

  std::string host = "10.0.0.3";
  std::string port = "4141";

  std::string topic = "points";
  std::string frame = "sensor";

  bool organize = true;
  bool useRosTime = false;
};


#endif
