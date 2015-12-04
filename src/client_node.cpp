/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy_client_ros/client_node.h>

#include <iostream>

// Simple application settings wrapper around boost property tree.
#include <quanergy_client_ros/settings.h>

ClientNode::ClientNode(int argc, char** argv)
  : settings_file("")
  , min(1.29)         // Distance filter
  , max(200.0)        // Distance filter
  , default_ring_range(0.0f)
  , default_ring_intensity(0)
  , ip("10.0.0.3")
  , port("4141")
  , topic("points")
  , frame("sensor")
  , organize(true)
  , useRosTime(false)
{
  for (int i = 0; i < quanergy::client::M8_NUM_LASERS; i++)
  {
    ring_range[i] = default_ring_range;
    ring_intensity[i] = default_ring_intensity;
  }

  ros::init(argc, argv, "Client");

  loadSettings(argc, argv);
  parseArgs(argc, argv);
}


void ClientNode::publish()
{
  SensorClient::Ptr grabber = SensorClient::Ptr(new SensorClient(ip, port));

  grabber->setMinimumDistanceThreshold(min);
  grabber->setMaximumDistanceThreshold(max);

  for (int i = 0; i < quanergy::client::M8_NUM_LASERS; i++)
  {
    grabber->setRingFilterMinimumRangeThreshold(i, ring_range[i]);
    grabber->setRingFilterMinimumIntensityThreshold(i, ring_intensity[i]);
  }
  
  SimplePublisher<quanergy::PointXYZIR> p(grabber, topic, frame, useRosTime);
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

void ClientNode::loadSettings(int argc, char ** argv)
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

    ip = settings.get("ClientRos.ip", ip);
    port = settings.get("ClientRos.port", port);

    topic = settings.get("ClientRos.topic", topic);
    frame = settings.get("ClientRos.frame", frame);

    organize = settings.get("ClientRos.organize", organize);
    useRosTime = settings.get("ClientRos.useRosTime", useRosTime);
  }
}


void ClientNode::parseArgs(int argc, char ** argv)
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


