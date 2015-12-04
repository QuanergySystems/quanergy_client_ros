/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#ifndef QUANERGY_CLIENT_M8_CLIENT_NODE_H
#define QUANERGY_CLIENT_M8_CLIENT_NODE_H

#include <string>
#include <cstdint>


#include "simple_m8_publisher.h"

struct M8ClientNode 
{

  M8ClientNode(int argc, char** argv);

  void publish();

private:

  void loadSettings(int argc, char ** argv);
  void parseArgs(int argc, char ** argv);

private:

  // Defaults, overridded by settings file, which is in turn
  // overridden by command-line options.

  std::string settings_file;

  // Distance filter

  float min;
  float max;

  // Ring filter
  // Only set by config file

  float default_ring_range;
  std::uint16_t default_ring_intensity;

  float ring_range[quanergy::client::M8_NUM_LASERS];
  std::uint16_t ring_intensity[quanergy::client::M8_NUM_LASERS];
  
  // M8 Client

  std::string ip;
  std::string port;

  std::string topic;
  std::string frame;

  bool organize;
  bool useRosTime;
};


#endif
