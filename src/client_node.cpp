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

#include <pcl/console/parse.h>

ClientNode::ClientNode(int argc, char** argv)
{
  ros::init(argc, argv, "Client");

  for (int i = 0; i < quanergy::client::M8_NUM_LASERS; i++)
  {
    settings_.ring_range[i] = settings_.default_ring_range;
    settings_.ring_intensity[i] = settings_.default_ring_intensity;
  }

  loadSettings(argc, argv);
  parseArgs(argc, argv);
}

void ClientNode::run()
{
  // create modules
  ClientType client(settings_.host, "4141", 100);
  ParserModuleType parser;
  DistanceFilter dFilter;
  RingIntensityFilter rFilter;
  ConverterType converter;
  SimplePublisher<quanergy::PointXYZIR> publisher(settings_.topic, settings_.useRosTime);

  // setup modules
  parser.get<0>().setFrameId(settings_.frame);
  parser.get<1>().setFrameId(settings_.frame);
  parser.get<2>().setFrameId(settings_.frame);
  dFilter.setMaximumDistanceThreshold(settings_.max);
  dFilter.setMinimumDistanceThreshold(settings_.min);
  for (int i = 0; i < quanergy::client::M8_NUM_LASERS; i++)
  {
    rFilter.setRingFilterMinimumRangeThreshold(i, settings_.ring_range[i]);
    rFilter.setRingFilterMinimumIntensityThreshold(i, settings_.ring_intensity[i]);
  }

  // connect modules
  std::vector<boost::signals2::connection> connections;
  connections.push_back(client.connect([&parser](const ClientType::ResultType& pc){ parser.slot(pc); }));
  connections.push_back(parser.connect([&dFilter](const ParserModuleType::ResultType& pc){ dFilter.slot(pc); }));
  connections.push_back(dFilter.connect([&rFilter](const DistanceFilter::ResultType& pc){ rFilter.slot(pc); }));
  connections.push_back(rFilter.connect([&converter](const RingIntensityFilter::ResultType& pc){ converter.slot(pc); }));
  connections.push_back(converter.connect([&publisher](const ConverterType::ResultType& pc){ publisher.slot(pc); }));

  // start client on a separate thread
  std::thread client_thread([&client, &publisher]
                            {
                              try
                              {
                                client.run();
                              }
                              catch (std::exception& e)
                              {
                                std::cerr << "Terminating after catching exception: " << e.what() << std::endl;
                                publisher.stop();
                              }
                            });


  // start publisher (blocks until stopped)
  publisher.run();

  // clean up
  client.stop();
  connections.clear();
  client_thread.join();
}

void ClientNode::loadSettings(int argc, char ** argv)
{
  // Is there a settings file specified?
  std::string settings_file;
  pcl::console::parse_argument (argc, argv, "-settings", settings_file);

  if (!settings_file.empty())
  {
    quanergy::Settings settings;

    settings.loadXML(settings_file);

    settings_.min = settings.get("DistanceFilter.min", settings_.min);
    settings_.max = settings.get("DistanceFilter.max", settings_.max);

    /// ring filter settings only relevant for M8
    for (int i = 0; i < quanergy::client::M8_NUM_LASERS; i++)
    {
      const std::string num = boost::lexical_cast<std::string>(i);

      std::string range_param = std::string("RingFilter.Range").append(num);
      settings_.ring_range[i] = settings.get(range_param, settings_.ring_range[i]);

      std::string intensity_param = std::string("RingFilter.Intensity").append(num);
      settings_.ring_intensity[i] = settings.get(intensity_param, settings_.ring_intensity[i]);
    }

    // support ip or host with host given priority
    settings_.host = settings.get("ClientRos.ip", settings_.host);
    settings_.host = settings.get("ClientRos.host", settings_.host);

    settings_.topic = settings.get("ClientRos.topic", settings_.topic);
    settings_.frame = settings.get("ClientRos.frame", settings_.frame);

    settings_.useRosTime = settings.get("ClientRos.useRosTime", settings_.useRosTime);
  }
}

void ClientNode::parseArgs(int argc, char ** argv)
{
  pcl::console::parse_argument (argc, argv, "-min", settings_.min);
  pcl::console::parse_argument (argc, argv, "-max", settings_.max);

  // support ip or host with host given priority
  pcl::console::parse_argument (argc, argv, "-ip", settings_.host);
  pcl::console::parse_argument (argc, argv, "-host", settings_.host);

  pcl::console::parse_argument (argc, argv, "-topic", settings_.topic);
  pcl::console::parse_argument (argc, argv, "-frame", settings_.frame);

  pcl::console::parse_argument (argc, argv, "-useRosTime", settings_.useRosTime);
}
