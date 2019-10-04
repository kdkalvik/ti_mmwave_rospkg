#include <fstream>
#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <zed_wrapper/start_svo_recording.h>
#include <zed_wrapper/stop_svo_recording.h>
#include <ti_mmwave_rospkg/RadarScan.h>
#include <ctime>
#include <sstream>
#include <thread>

std::fstream fout;
ros::NodeHandle *global_n;
int total_points = 0;
std::string dca1000_config_file;
std::string data_path;

void start_record()
{
  if (std::system(("DCA1000EVM_CLI_Control start_record " + dca1000_config_file + " -q").c_str()))
  {
    ROS_ERROR("Failed to call start_record, raw data capture from ti device");
    ros::shutdown();
  }
}

void ShutdownCallback(const ros::TimerEvent&)
{
  ros::ServiceClient stop_svo_client = global_n->serviceClient<zed_wrapper::stop_svo_recording>("/zed/zed_node/stop_svo_recording");
  zed_wrapper::stop_svo_recording stop_svo_srv;

  //Stop ZED recording
  if (!stop_svo_client.call(stop_svo_srv))
  {
    ROS_ERROR("Failed to call service stop_svo_recording");
  }

  //Close csv file
  fout.close();
  ros::shutdown();
  if (std::system(("DCA1000EVM_CLI_Control stop_record " + dca1000_config_file).c_str()))
  {
    ROS_ERROR("Failed to call stop_record, raw data capture from ti device");
  }

  if (std::system(("cp ~/catkin_ws/src/ti_mmwave_rospkg/adc_data_Raw_0.bin " + data_path).c_str()))
  {
    ROS_ERROR("Failed to call stop_record, raw data capture from ti device");
  }

  if (std::system(("cp ~/catkin_ws/src/ti_mmwave_rospkg/adc_data_Raw_LogFile.csv " + data_path).c_str()))
  {
    ROS_ERROR("Failed to call stop_record, raw data capture from ti device");
  }
}

void CheckSubCallback(const ros::TimerEvent&)
{
  if (total_points == 0)
  {
    ros::ServiceClient stop_svo_client = global_n->serviceClient<zed_wrapper::stop_svo_recording>("/zed/zed_node/stop_svo_recording");
    zed_wrapper::stop_svo_recording stop_svo_srv;

    //Stop ZED recording
    ROS_ERROR("Shutting Down /ti_mmwave/radar_scan not publishing !!");
    if (!stop_svo_client.call(stop_svo_srv))
    {
      ROS_ERROR("Failed to call service stop_svo_recording");
    }

    //Close csv file
    fout.close();
    ros::shutdown();
    if (std::system(("DCA1000EVM_CLI_Control stop_record " + dca1000_config_file).c_str()))
    {
      ROS_ERROR("Failed to call stop_record, raw data capture from ti device");
    }
  }
}

void SerializeCallback(const ti_mmwave_rospkg::RadarScan& msg)
{
  //Write all points of a frame in a single line
  if (msg.point_id == 0)
    fout << "\n";

  fout << msg.header.stamp.sec << msg.header.stamp.nsec << ",";
  fout << msg.point_id << ",";
  fout << msg.x << ",";
  fout << msg.y << ",";
  fout << msg.z << ",";
  fout << msg.range << ",";
  fout << msg.velocity << ",";
  fout << msg.doppler_bin << ",";
  fout << msg.bearing << ",";
  fout << msg.intensity << ",";
  total_points += 1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mmWaveDataRecorder");
  ros::NodeHandle n;
  global_n = &n;

  std::string user, dataset_path;
  double rec_time;
  if (!n.getParam("/ti_mmwave/user", user))
  {
    ROS_ERROR("Failed to get param 'user'");
    return 1;
  }

  if (!n.getParam("/ti_mmwave/dataset_path", dataset_path))
  {
    ROS_ERROR("Failed to get param 'dataset_path'");
    return 1;
  }

  if (!n.getParam("/ti_mmwave/rec_time", rec_time))
  {
    ROS_ERROR("Failed to get param 'rec_time'");
    return 1;
  }

  if (!n.getParam("/ti_mmwave/dca1000_config_file", dca1000_config_file))
  {
    ROS_ERROR("Failed to get param 'dca1000_config_file'");
    return 1;
  }

  ros::ServiceClient start_svo_client = n.serviceClient<zed_wrapper::start_svo_recording>("/zed/zed_node/start_svo_recording");
  ros::Subscriber sub = n.subscribe("/ti_mmwave/radar_scan", 10000, SerializeCallback);

  zed_wrapper::start_svo_recording start_svo_srv;

  time_t now = time(0);
  tm *ltm = localtime(&now);

  std::stringstream filename;
  filename << dataset_path << user << "_"
           << ltm->tm_mday << "_"
           << 1 + ltm->tm_mon << "_"
           << 1 + ltm->tm_hour
           << 1 + ltm->tm_min;
  data_path = filename.str();

  if (std::system(("mkdir " + filename.str()).c_str()))
  {
   ROS_ERROR("Failed to create folder for data");
   fout.close();
   return 1;
  }

  start_svo_srv.request.svo_filename = filename.str()+"/video.svo";
  fout.open(filename.str()+"/points.csv", std::ios::out | std::ios::app);

  //Wait for ZED and mmwave nodes to startup
  ros::Duration(5).sleep();

  //Start raw data capture from ti device
  if (std::system(("DCA1000EVM_CLI_Control fpga " + dca1000_config_file).c_str()))
  {
    ROS_ERROR("Failed to call fpga, raw data capture from ti device");
    fout.close();
    return 1;
  }

  if (std::system(("DCA1000EVM_CLI_Control record " + dca1000_config_file).c_str()))
  {
    ROS_ERROR("Failed to call record, raw data capture from ti device");
    fout.close();
    return 1;
  }
  std::thread t1(start_record);

  //Start ZED recording
  if (!start_svo_client.call(start_svo_srv))
  {
    ROS_ERROR("Failed to call service start_svo_recording");
    fout.close();

    if (std::system(("DCA1000EVM_CLI_Control stop_record " + dca1000_config_file).c_str()))
    {
      ROS_ERROR("Failed to call stop_record, raw data capture from ti device");
    }
    return 1;
  }

  ros::Timer timer = n.createTimer(ros::Duration(rec_time), ShutdownCallback, true);
  ros::Timer check_timer = n.createTimer(ros::Duration(1), CheckSubCallback, true);

  while (ros::ok())
  {
    ros::spinOnce();
  }

  ros::TimerEvent tmp;
  ShutdownCallback(tmp);

  return 0;
}
