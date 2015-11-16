/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include "m8_client_node.h"

#include <iostream>

// Simple application settings wrapper around boost property tree.
#include "settings.h"

M8ClientNode::M8ClientNode(int argc, char** argv)
  : settings_file("")
  , min(1.29)         // Distance filter
  , max(200.0)        // Distance filter
  , default_ring_range(0.0f)
  , default_ring_intensity(0)
  , ip("10.0.0.3")
  , port("4141")
  , topic("m8_points")
  , frame("m8")
  , organize(true)
  , useRosTime(false)
{
  for (int i = 0; i < quanergy::client::M8_NUM_LASERS; i++)
  {
    ring_range[i] = default_ring_range;
    ring_intensity[i] = default_ring_intensity;
  }

  ros::init(argc, argv, "M8Client");

  loadSettings(argc, argv);
  parseArgs(argc, argv);
}


void M8ClientNode::publish()
{
  M8SensorClient::Ptr grabber = M8SensorClient::Ptr(new M8SensorClient(ip, port));

  grabber->setMinimumDistanceThreshold(min);
  grabber->setMaximumDistanceThreshold(max);

  for (int i = 0; i < quanergy::client::M8_NUM_LASERS; i++)
  {
    grabber->setRingFilterMinimumRangeThreshold(i, ring_range[i]);
    grabber->setRingFilterMinimumIntensityThreshold(i, ring_intensity[i]);
  }
  
  SimpleM8Publisher<quanergy::PointXYZIR> p(grabber, topic, frame, useRosTime);
  try
  {
    p.run();
  }
  catch (quanergy::client::SocketBindError& e)
  {
    std::cout << "Unable to bind to socket; shutting down" << std::endl;
  }
  catch (quanergy::client::SocketReadError& e)
  {
    std::cout << "Socket connection dropped; shutting down" << std::endl;
  }
}

void M8ClientNode::loadSettings(int argc, char ** argv)
{
  // Is there a settings file specified?

  pcl::console::parse_argument (argc, argv, "-settings", settings_file);

  if (!settings_file.empty())
  {
    quanergy::Settings settings;

    settings.loadXML(settings_file);

    min = settings.get("DistanceFilter.min", min);
    max = settings.get("DistanceFilter.max", max);

    for (int i = 0; i < quanergy::client::M8_NUM_LASERS; i++)
    {
      const std::string num = boost::lexical_cast<std::string>(i);

      std::string range_param = std::string("RingFilter.Range").append(num);
      ring_range[i] = settings.get(range_param, ring_range[i]);

      std::string intensity_param = std::string("RingFilter.Intensity").append(num);
      ring_intensity[i] = settings.get(intensity_param, ring_intensity[i]);
    }

    ip = settings.get("M8ClientRos.ip", ip);
    port = settings.get("M8ClientRos.port", port);

    topic = settings.get("M8ClientRos.topic", topic);
    frame = settings.get("M8ClientRos.frame", frame);

    organize = settings.get("M8ClientRos.organize", organize);
    useRosTime = settings.get("M8ClientRos.useRosTime", useRosTime);
  }
}


void M8ClientNode::parseArgs(int argc, char ** argv)
{
  pcl::console::parse_argument (argc, argv, "-min", min);
  pcl::console::parse_argument (argc, argv, "-max", max);

  pcl::console::parse_argument (argc, argv, "-ip", ip);
  pcl::console::parse_argument (argc, argv, "-port", port);

  pcl::console::parse_argument (argc, argv, "-topic", topic);
  pcl::console::parse_argument (argc, argv, "-frame", frame);

  pcl::console::parse_argument (argc, argv, "-organize", organize);
  pcl::console::parse_argument (argc, argv, "-useRosTime", useRosTime);
}


