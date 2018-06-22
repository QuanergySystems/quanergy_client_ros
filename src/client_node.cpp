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

int ClientNode::returnFromString(const std::string& r)
{
  int ret;

  if (r == "all")
  {
    ret = quanergy::client::ALL_RETURNS;
    return ret;
  }

  if (r == "all_separate_topics")
  {
    separate_return_topics_ = true;
    return quanergy::client::ALL_RETURNS;
  }

  // Verify argument contains only digits
  if (!r.empty() && std::all_of(r.begin(), r.end(), ::isdigit))
  {
    ret = std::atoi(r.c_str());
    if (ret < 0 || ret >= quanergy::client::M8_NUM_RETURNS)
    {
      throw std::invalid_argument("Invalid return selection");
    }
  }
  else
  {
    throw std::invalid_argument("Invalid return selection");
  }

  return ret;
}

std::string stringFromReturn(int r)
{
  std::string ret;

  if (r == quanergy::client::ALL_RETURNS)
  {
    ret = "all";
  }
  else if (r >= 0 && r < quanergy::client::M8_NUM_RETURNS)
  {
    ret = std::to_string(r);
  }
  else
  {
    throw std::invalid_argument("Invalid return selection");
  }

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
           " [--frame <frame>] [--useRosTime 0 | 1] [--return <number> | all | all_separate_topics] [--min-cloud <min>] [--max-cloud <max>] [-h | --help]" << std::endl
        << std::endl
        << "    --settings                      settings file; these settings are overridden by commandline arguments" << std::endl
        << "    --host                          hostname or IP address of the sensor" << std::endl
        << "    --encoder-amplitude-correction  amplitude when applying encoder correction" << std::endl
        << "    --encoder-phase-correction      phase offset (in rad) when applying encoder correction" << std::endl
        << "    --min                           minimum range for filtering" << std::endl
        << "    --max                           maximum range for filtering" << std::endl
        << "    --topic                         ROS topic for publishing the point cloud. If 'all_separate_topics' is chosen for --return, then a topic will be created for each return number with the return number appended to the topic name" << std::endl
        << "    --frame                         frame ID for the point cloud" << std::endl
        << "    --useRosTime                    boolean setting for point cloud time; uses sensor time if false" << std::endl
        << "    --return                        return selection for multiple return M8 sensors. Options are 0, 1, 2, all, or all_separate_topics. If 'all' is chosen, then all 3 returns will be returned on one topic. If 'all_separate_topics' is chosen, then there will be one topic per return" << std::endl
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
  std::vector<SensorPipelineModules::Ptr> sensor_pipelines;
  std::vector<std::thread> pipeline_threads;

  // Run pipeline(s) in their own thread(s)
  if (separate_return_topics_)
  {
    int pipeline_count = quanergy::client::M8_NUM_RETURNS;
    sensor_pipelines.reserve(pipeline_count);
    for (int i=0; i<pipeline_count; ++i)
    {
      sensor_pipelines.emplace_back(
        // Add the return number to the ROS topic name
        new SensorPipelineModules(settings_, i, client, true)
      );
      pipeline_threads.emplace_back(
        [&sensor_pipelines, i, &client, this]
        {
          sensor_pipelines[i]->run();
          std::lock_guard<std::mutex> lock(client_mutex_);
          client.stop();
        }
      );
    }
  }
  else
  {
    sensor_pipelines.emplace_back(
      // Don't add anything to the ROS topic name
      new SensorPipelineModules(
        settings_, settings_.return_selection, client)
    );
    pipeline_threads.emplace_back(
      [&sensor_pipelines, &client, this]
      {
        sensor_pipelines[0]->run();
        std::lock_guard<std::mutex> lock(client_mutex_);
        client.stop();
      }
    );
  }

  waitForPublisherStartup(sensor_pipelines);

  // start client
  try
  {
    client.run();
  }
  catch (std::exception& e)
  {
    std::cerr << "Terminating after catching exception: " 
              << e.what() 
              << std::endl;
  }

  // Clean up ROS
  ros::shutdown();
  
  for (auto &thread : pipeline_threads)
  {
    thread.join();
  }
  pipeline_threads.clear();
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

void ClientNode::waitForPublisherStartup(
  const std::vector<ClientNode::SensorPipelineModules::Ptr>& pipelines
)
{
  if (pipelines.empty())
  {
    return;
  }

  bool waiting = true;
  while (waiting)
  {
    bool ready = true;
    for (const SensorPipelineModules::Ptr& pipeline : pipelines)
    {
      if (!pipeline->publisher.ready())
      {
        ready = false;
        break;
      }
    }

    if (!ready)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    else
    {
      waiting = false;
    }
  }
}

// SensorPipelineModules 

ClientNode::SensorPipelineModules::SensorPipelineModules(
  const ClientNode::Settings &settings,
  int return_selection,
  ClientNode::ClientType &client,
  bool add_return_number_to_topic /* = false */) 
  : publisher(settings.useRosTime)
{
  ros_topic_name_ = settings.topic;
  if (add_return_number_to_topic && return_selection >= 0)
  {
    ros_topic_name_ += std::to_string(return_selection);
  }

  encoder_corrector.setParams(settings.amplitude, settings.phase);

  // Setup modules

  // Parsers
  auto &parser00 = parser.get<PARSER_00_INDEX>();
  auto &parser01 = parser.get<PARSER_01_INDEX>();
  auto &parser04 = parser.get<PARSER_04_INDEX>();

  // Parser 00
  parser00.setFrameId(settings.frame);
  parser00.setReturnSelection(return_selection);
  parser00.setCloudSizeLimits(
    settings.minCloudSize,
    settings.maxCloudSize
  );

  // Parser 01
  parser01.setFrameId(settings.frame);

  // Parser 04
  parser04.setFrameId(settings.frame);
  parser04.setCloudSizeLimits(
    settings.minCloudSize, 
    settings.maxCloudSize
  );

  // Filters

  // Distance Filter
  distance_filter.setMaximumDistanceThreshold(settings.max);
  distance_filter.setMinimumDistanceThreshold(settings.min);
  for (int i = 0; i < quanergy::client::M8_NUM_LASERS; ++i)
  {
    ring_intensity_filter.setRingFilterMinimumRangeThreshold(
      i, settings.ring_range[i]
    );
    ring_intensity_filter.setRingFilterMinimumIntensityThreshold(
      i, settings.ring_intensity[i]
    );
  }

  // Connect modules
  // Client to Parser
  connections.push_back(
    client.connect(
      [this](const ClientType::ResultType& pc){ parser.slot(pc); }
    )
  );
  // Parser to Encoder Corrector
  connections.push_back(
    parser.connect(
      [this](const ParserModuleType::ResultType& pc)
      { encoder_corrector.slot(pc); }
    )
  );
  // Encoder Corrector to Distance Filter
  connections.push_back(
    encoder_corrector.connect(
      [this](const EncoderAngleCalibrationType::ResultType& pc)
      { distance_filter.slot(pc); }
    )
  );
  // Distance Filter to Ring Intensity Filter
  connections.push_back(
    distance_filter.connect(
      [this](const DistanceFilter::ResultType& pc)
      { ring_intensity_filter.slot(pc); }
    )
  );
  // Ring Intensity Filter to Polar->Cartesian Converter
  connections.push_back(
    ring_intensity_filter.connect(
      [this](const RingIntensityFilter::ResultType& pc)
      { cartesian_converter.slot(pc); }
    )
  );
  // Polar->Cartesian Converter to Publisher
  connections.push_back(
    cartesian_converter.connect(
      [this](const ConverterType::ResultType& pc)
      { publisher.slot(pc); }
    )
  );
}

ClientNode::SensorPipelineModules::~SensorPipelineModules()
{
  // Clean up
  for (auto &connection : connections)
  {
    connection.disconnect();
  }
  connections.clear();
}

void ClientNode::SensorPipelineModules::run()
{     
  // Blocks until stopped 
  publisher.run(ros_topic_name_);
}


