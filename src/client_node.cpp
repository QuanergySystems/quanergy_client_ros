/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

// publisher module
#include <quanergy_client_ros/simple_publisher.h>

#include <quanergy/client/version.h>

#if (QUANERGY_CLIENT_VERSION/100000 != 5)
  #error Incompatible Quanergy Client Version; looking for v5.x.x
#endif

// console parser
#include <boost/program_options.hpp>

// TCP client for sensor
#include <quanergy/client/sensor_client.h>

// sensor pipeline
#include <quanergy/pipelines/sensor_pipeline.h>

// async module for multithreading
#include <quanergy/pipelines/async.h>

// settings specific to ROS
struct RosNodeSettings
{
  // Whether or not to publish each return on a separate topic
  // This reuses SensorPipeline's return_selection with an additional option
  bool separate_return_topics = false;

  // ROS topic to publish on
  std::string topic = "points";

  // determines whether to use ROS Time in the ROS Msg.
  bool use_ros_time = false;

  /** \brief loads settings from SettingsFileLoader */
  void load(const quanergy::pipeline::SettingsFileLoader& settings);
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "quanergy_client_ros");
  ros::NodeHandle nh;

  namespace po = boost::program_options;

  po::options_description description("Quanergy Client ROS Node");
  const po::positional_options_description p; // empty positional options

  quanergy::pipeline::SensorPipelineSettings pipeline_settings;
  RosNodeSettings ros_node_settings;
  std::string return_string;
  std::vector<float> correct_params;

  // port
  std::string port = "4141";

  description.add_options()
    ("help,h", "Display this help message.")
    ("settings-file,s", po::value<std::string>(),
      "Settings file. Setting file values override defaults and command line arguments override the settings file.")
    ("topic,t", po::value<std::string>(&ros_node_settings.topic)->
      default_value(ros_node_settings.topic), 
      "ROS topic for publishing the point cloud. If 'all_separate_topics' is chosen for --return, " 
      "a topic will be created for each return number with the return number appended to the topic name.")
    ("use-ros-time", po::bool_switch(&ros_node_settings.use_ros_time),
      "flag determining whether to use ROS time in message; uses sensor time otherwise,")
    ("host", po::value<std::string>(&pipeline_settings.host),
      "Host name or IP of the sensor.")
    ("frame,f", po::value<std::string>(&pipeline_settings.frame)->
      default_value(pipeline_settings.frame),
      "Frame name inserted in the point cloud.")
    ("return,r", po::value<std::string>(&return_string),
      "Return selection (M-series only) - "
      "Options are 0, 1, 2, all, or all_separate_topics. For 3 return packets, 'all' creates an unorganized point cloud "
      "and 'all_separate_topics' creates 3 separate organized point clouds on their own topics. "
      "For single return, explicitly setting a value produces an error if the selection doesn't match the packet.")
    ("calibrate", po::bool_switch(&pipeline_settings.calibrate),
      "Flag indicating encoder calibration should be performed and applied to outgoing points; M-series only.")
    ("frame-rate", po::value<double>(&pipeline_settings.frame_rate)->
      default_value(pipeline_settings.frame_rate),
      "Frame rate used when peforming encoder calibration; M-series only.")
    ("manual-correct", po::value<std::vector<float>>(&correct_params)->multitoken()->value_name("amplitude phase"),
      "Correct encoder error with user defined values. Both amplitude and phase are in radians; M-series only.")
    ("min-distance", po::value<float>(&pipeline_settings.min_distance)->
      default_value(pipeline_settings.min_distance),
      "minimum distance (inclusive) for distance filtering.")
    ("max-distance", po::value<float>(&pipeline_settings.max_distance)->
      default_value(pipeline_settings.max_distance),
      "maximum distance (inclusive) for distance filtering.")
    ("min-cloud-size", po::value<std::int32_t>(&pipeline_settings.min_cloud_size)->
      default_value(pipeline_settings.min_cloud_size),
      "minimum cloud size; produces an error and ignores clouds smaller than this.")
    ("max-cloud-size", po::value<std::int32_t>(&pipeline_settings.max_cloud_size)->
      default_value(pipeline_settings.max_cloud_size),
      "maximum cloud size; produces an error and ignores clouds larger than this.");

  try
  {
    // load the command line options into the variables map
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(description).positional(p).run(), vm);

    if (vm.count("help"))
    {
      std::cout << description << std::endl;
      return 0;
    }

    // if there is a settings file, load that before notifying (which fills the variables)
    if (vm.count("settings-file"))
    {
      std::string settings_file = vm["settings-file"].as<std::string>();
      quanergy::pipeline::SettingsFileLoader file_loader;
      file_loader.loadXML(settings_file);
      ros_node_settings.load(file_loader);

      if (ros_node_settings.separate_return_topics)
      {
        // we'll deal with this later but we need to change the value in settings
        // since pipeline_settings won't know how to handle it
        file_loader.put("Settings.return", pipeline_settings.stringFromReturn(0));
      }

      pipeline_settings.load(file_loader);
    }

    // notify; this stores command line options in associated variables
    po::notify(vm);

    // let the user know if there is no host value
    if (pipeline_settings.host.empty())
    {
      std::cout << "No host provided" << std::endl;
      std::cout << description << std::endl;
      return -1;
    }

    // handle return selection
    if (!return_string.empty())
    {
      pipeline_settings.return_selection_set = true;
      if (return_string == "all_separate_topics")
      {
        ros_node_settings.separate_return_topics = true;
        pipeline_settings.return_selection = 0;
      }
      else
      {
        pipeline_settings.return_selection = pipeline_settings.returnFromString(return_string);
      }
    }

    // handle encoder correction parameters
    if (!correct_params.empty())
    {
      if (correct_params.size() == 2)
      {
        pipeline_settings.override_encoder_params = true;
        pipeline_settings.amplitude = correct_params[0];
        pipeline_settings.phase = correct_params[1];
      }
      else
      {
        std::cout << "Manual encoder correction expects exactly 2 paramters: amplitude and phase" << std::endl;
        std::cout << description << std::endl;
        return -1;
      }
    }
  }
  catch (po::error& e)
  {
    std::cout << "Boost Program Options Error: " << e.what() << std::endl << std::endl;
    std::cout << description << std::endl;
    return -1;
  }
  catch (std::exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
    return -2;
  }


  // create client to get raw packets from the sensor
  quanergy::client::SensorClient client(pipeline_settings.host, port, 100);

  // create list (because it is noncopyable) of asyncs that will use if all_separate_topics so processing can happen on separate threads
  using Async = quanergy::pipeline::AsyncModule<std::shared_ptr<std::vector<char>>>;
  std::list<Async> asyncs;

  // create list (because it is noncopyable) of pipelines to produce point cloud from raw packets
  // we may need multiple for separate_return_topics
  std::list<quanergy::pipeline::SensorPipeline> pipelines;

  // create list (because it is noncopyable) of ROS publishers to consume point clouds and publish them
  // we may need multiple for separate_return_topics
  std::list<SimplePublisher<quanergy::PointXYZIR>> publishers;

  // create vector of threads for publisher(s)
  // we may need multiple for separate_return_topics
  std::vector<std::thread> publisher_threads;

  // store connections for cleaner shutdown
  std::vector<boost::signals2::connection> connections;

  // Some synchronization stuff
  std::mutex client_mutex;
  std::atomic<int> publishers_count = {0};
  std::condition_variable publisher_cv;

  // connect things and run publisher(s) on their own thread(s)
  int pipeline_count = 1;
  if (ros_node_settings.separate_return_topics)
  {
    pipeline_count = quanergy::client::M_SERIES_NUM_RETURNS;
  }

  for (int i = 0; i < pipeline_count; ++i)
  {
    // set return for pipeline and define topic, if needed
    std::string topic = ros_node_settings.topic;
    if (ros_node_settings.separate_return_topics)
    {
      pipeline_settings.return_selection = i;
      topic += std::to_string(i);
    }

    // create pipeline and get reference to it
    pipelines.emplace_back(pipeline_settings);
    auto& pipeline = pipelines.back();

    // create publisher and get reference to it
    publishers.emplace_back(nh, topic, ros_node_settings.use_ros_time);
    auto& publisher = publishers.back();

    if (ros_node_settings.separate_return_topics)
    {
      // create async to put pipelines on separate threads
      // need larger queue because the parser collects ~100 packets to assemble into point cloud
      asyncs.emplace_back(100); 
      auto& async = asyncs.back();

      // connect client to async
      connections.push_back(client.connect(
        [&async](const std::shared_ptr<std::vector<char>>& packet){ async.slot(packet); }
      ));

      // connect async to pipeline
      connections.push_back(async.connect(
        [&pipeline](const std::shared_ptr<std::vector<char>>& packet){ pipeline.slot(packet); }
      ));
    }
    else
    {
      // connect client to pipeline
      connections.push_back(client.connect(
        [&pipeline](const std::shared_ptr<std::vector<char>>& packet){ pipeline.slot(packet); }
      ));
    }

    // connect the pipeline to the publisher
    connections.push_back(pipeline.connect(
        [&publisher](const boost::shared_ptr<pcl::PointCloud<quanergy::PointXYZIR>>& pc){ publisher.slot(pc); }
    ));

    // create publisher thread
    publisher_threads.emplace_back(
      [&publisher, &client_mutex, &client, &publishers_count, &publisher_cv]
      {
        std::unique_lock<std::mutex> lock(client_mutex);
        ++publishers_count;
        lock.unlock();
        publisher_cv.notify_one();

        publisher.run();
        lock.lock();
        client.stop();
      }
    );
  }

  // this makes sure the publishers have started; without it, sometimes the client starts first (due to OS scheduling)
  // and then we get a bunch of warning messages related to data coming in without being consumed
  {
    std::unique_lock<std::mutex> lock(client_mutex);
    publisher_cv.wait(lock, [&publishers_count, pipeline_count]{ return publishers_count == pipeline_count; });
  }

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


  // Clean up
  ros::shutdown();
  
  connections.clear();

  for (auto &thread : publisher_threads)
  {
    thread.join();
  }

  publisher_threads.clear();

  return (0);
}

void RosNodeSettings::load(const quanergy::pipeline::SettingsFileLoader& settings)
{
  auto r = settings.get_optional<std::string>("Settings.return");
  if (r && *r == "all_separate_topics")
  {
    separate_return_topics = true;
  }

  topic = settings.get("Settings.RosNode.topic", topic);

  use_ros_time = settings.get("Settings.RosNode.useRosTime", use_ros_time);
}
