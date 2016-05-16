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

quanergy::client::ReturnSelection returnFromString(const std::string& r)
{
  quanergy::client::ReturnSelection ret;

  if (r == "max")
    ret = quanergy::client::ReturnSelection::MAX;
  else if (r == "first")
    ret = quanergy::client::ReturnSelection::FIRST;
  else if (r == "last")
    ret = quanergy::client::ReturnSelection::LAST;
  else if (r == "all")
    ret = quanergy::client::ReturnSelection::ALL;
  else
    throw std::invalid_argument("Invalid return selection");

  return ret;
}

std::string stringFromReturn(quanergy::client::ReturnSelection r)
{
  std::string ret;

  if (r == quanergy::client::ReturnSelection::MAX)
    ret = "max";
  else if (r == quanergy::client::ReturnSelection::FIRST)
    ret = "first";
  else if (r == quanergy::client::ReturnSelection::LAST)
    ret = "last";
  else if (r == quanergy::client::ReturnSelection::ALL)
    ret = "all";
  else
    throw std::invalid_argument("Invalid return selection");

  return ret;
}

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

bool ClientNode::checkArgs(int argc, char** argv)
{
  if (pcl::console::find_switch(argc, argv, "-h") ||
      pcl::console::find_switch(argc, argv, "--help") ||
      (!pcl::console::find_switch(argc, argv, "--settings") &&
       !pcl::console::find_switch(argc, argv, "--host")))
  {
    std::cout << "usage: " << argv[0]
        << " [--settings <file>] [--host <host>] [--encoder-amplitude-correction <amplitude>] [--encoder-phase-correction <phase>] [--min <min>] [--max <max>] [--topic <topic>]"
           " [--frame <frame>] [--useRosTime 0 | 1] [--return max | first | last | all] [--min-cloud <min>] [--max-cloud <max>] [-h | --help]" << std::endl
        << std::endl
        << "    --settings                      settings file; these settings are overridden by commandline arguments" << std::endl
        << "    --host                          hostname or IP address of the sensor" << std::endl
        << "    --encoder-amplitude-correction  amplitude when applying encoder correction" << std::endl
        << "    --encoder-phase-correction      phase offset (in rad) when applying encoder correction" << std::endl
        << "    --min                           minimum range for filtering" << std::endl
        << "    --max                           maximum range for filtering" << std::endl
        << "    --topic                         ROS topic for publishing the point cloud" << std::endl
        << "    --frame                         frame ID for the point cloud" << std::endl
        << "    --useRosTime                    boolean setting for point cloud time; uses sensor time if false" << std::endl
        << "    --return                        return selection for multiple return M8 sensors" << std::endl
        << "    --min-cloud                     minimum number of points for a valid cloud" << std::endl
        << "    --max-cloud                     maximum number of points allowed in a cloud" << std::endl
        << "-h, --help                          show this help and exit" << std::endl;
    return false;
  }
  return true;
}

void ClientNode::run()
{
  // create modules
  ClientType client(settings_.host, settings_.port, 100);
  ParserModuleType parser;
  DistanceFilter dFilter;
  RingIntensityFilter rFilter;
  ConverterType converter;
  SimplePublisher<quanergy::PointXYZIR> publisher(settings_.topic, settings_.useRosTime);
  EncoderAngleCalibrationType encoder_corrector;
  encoder_corrector.setParams(settings_.amplitude, settings_.phase);

  // setup modules
  parser.get<0>().setFrameId(settings_.frame);
  parser.get<0>().setCloudSizeLimits(settings_.minCloudSize,settings_.maxCloudSize);
  parser.get<1>().setFrameId(settings_.frame);
  parser.get<1>().setReturnSelection(settings_.return_selection);
  parser.get<1>().setCloudSizeLimits(settings_.minCloudSize,settings_.maxCloudSize);
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
  connections.push_back(parser.connect([&encoder_corrector](const ParserModuleType::ResultType& pc){ encoder_corrector.slot(pc); }));
  connections.push_back(encoder_corrector.connect([&dFilter](const EncoderAngleCalibrationType::ResultType& pc){ dFilter.slot(pc); }));
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
  pcl::console::parse_argument (argc, argv, "--settings", settings_file);

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

    settings_.amplitude = settings.get("EncoderCorrection.amplitude", settings_.amplitude);
    settings_.phase = settings.get("EncoderCorrection.phase", settings_.phase);

    settings_.host = settings.get("ClientRos.host", settings_.host);

    settings_.topic = settings.get("ClientRos.topic", settings_.topic);
    settings_.frame = settings.get("ClientRos.frame", settings_.frame);

    settings_.useRosTime = settings.get("ClientRos.useRosTime", settings_.useRosTime);

    std::string r = stringFromReturn(settings_.return_selection);
    r = settings.get("ClientRos.return", r);
    settings_.return_selection = returnFromString(r);

    settings_.minCloudSize = settings.get("ClientRos.minCloudSize", settings_.minCloudSize);
    settings_.maxCloudSize = settings.get("ClientRos.maxCloudSize", settings_.maxCloudSize);
  }
}

void ClientNode::parseArgs(int argc, char ** argv)
{
  pcl::console::parse_argument (argc, argv, "--min", settings_.min);
  pcl::console::parse_argument (argc, argv, "--max", settings_.max);

  pcl::console::parse_argument (argc, argv, "--host", settings_.host);

  pcl::console::parse_argument(argc, argv, "--encoder-amplitude-correction",
                               settings_.amplitude);
  pcl::console::parse_argument(argc, argv, "--encoder-phase-correction",
                               settings_.phase);

  pcl::console::parse_argument (argc, argv, "--topic", settings_.topic);
  pcl::console::parse_argument (argc, argv, "--frame", settings_.frame);

  pcl::console::parse_argument (argc, argv, "--useRosTime", settings_.useRosTime);

  std::string r = stringFromReturn(settings_.return_selection);
  pcl::console::parse_argument (argc, argv, "--return", r);
  settings_.return_selection = returnFromString(r);

  pcl::console::parse_argument (argc, argv, "--min-cloud", settings_.minCloudSize);
  pcl::console::parse_argument (argc, argv, "--max-cloud", settings_.maxCloudSize);
}
