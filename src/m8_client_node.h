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

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <quanergy/common/pointcloud_types.h>

#include <quanergy/parsers/failover_client.h>
#include <quanergy/parsers/pointcloud_generator_01.h>
#include <quanergy/parsers/pointcloud_generator_00.h>

#include <quanergy/modules/polar_to_cart_converter.h>
#include <quanergy/modules/distance_filter.h>
#include <quanergy/modules/ring_intensity_filter.h>

// Simple application settings wrapper around boost property tree.
#include "settings.h"

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
