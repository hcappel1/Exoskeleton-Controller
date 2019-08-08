#include <ros/ros.h>
#include <apptronik_msgs/Float32Stamped.h>
#include <apptronik_srvs/Float32.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <boost/bind.hpp>
// #include <boost/tokenizer.hpp>
// #include <boost/algorithm/string.hpp>
// #include <boost/lexical_cast.hpp>

namespace p2_test_utils
{
  std::vector<double> sum_list;
  std::vector<int> sum_count;

  void dataCallback(const apptronik_msgs::Float32Stamped::ConstPtr& msg, int channel_idx)
  {
    sum_list[channel_idx] += msg->data;
    sum_count[channel_idx]++;
  }
}

using namespace p2_test_utils;

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "auto_calibrate_ati_sensors");
  ros::NodeHandle nh;

  std::vector<std::string> slave_names = {"Axon_proto_v1"};
  // std::vector<std::string> slave_names = {"Axon_v3_linear", "Axon_v3_linear_3"};
  std::vector<std::string> topic_names = {"actuator__force__N",
                                          "ati__Fx__N",
                                          "ati__Fy__N",
                                          "ati__Fz__N",
                                          "ati__Tx__Nm",
                                          "ati__Ty__Nm",
                                          "ati__Tz__Nm"};
  std::vector<std::string> service_names = {"Sensing__Force__APS_Offset_N",
                                            "Sensing__Force__ATI_FX_Offset_N",
                                            "Sensing__Force__ATI_FY_Offset_N",
                                            "Sensing__Force__ATI_FZ_Offset_N",
                                            "Sensing__Torque__ATI_TX_Offset_Nm",
                                            "Sensing__Torque__ATI_TY_Offset_Nm",
                                            "Sensing__Torque__ATI_TZ_Offset_Nm"};


  std::vector<ros::Subscriber> sub_list = std::vector<ros::Subscriber>();

  int sub_idx = 0;
  for(int i = 0; i < slave_names.size(); i++)
  {
    for(int ii = 0; ii < topic_names.size(); ii++)
    {
      std::string full_topic = "/" + slave_names[i] + "/miso/" + topic_names[ii];
      sub_list.push_back(nh.subscribe<apptronik_msgs::Float32Stamped>(full_topic, 1000, boost::bind(dataCallback, _1, sub_idx)));
      sub_idx++;
    }
  }

  // keeps track of sum of values
  sum_list  = std::vector<double>(sub_list.size(), 0); // fill constructor
  sum_count = std::vector<int>(sub_list.size(), 0);    // fill constructor

  ros::Rate r(100);
  // spin for 2 seconds
  for(int i = 0; i < 200; i++)
  {
    ros::spinOnce();
    r.sleep();
  }


  // compute averages and adjust
  sub_idx = 0;
  for(int i = 0; i < slave_names.size(); i++)
  {
    for(int ii = 0; ii < topic_names.size(); ii++)
    {
      
      float avg_val = sum_list[sub_idx] / sum_count[sub_idx];

      std::string full_get_service = "/" + slave_names[i] + "/" + service_names[ii] + "/" + "get";
      std::string full_set_service = "/" + slave_names[i] + "/" + service_names[ii] + "/" + "set";

      ros::ServiceClient client = nh.serviceClient<apptronik_srvs::Float32>(full_get_service);
      apptronik_srvs::Float32 srv = apptronik_srvs::Float32();

      float prev_offset = 0.0;
      if (client.call(srv))
      {

        prev_offset = srv.response.get_data;
        std::cout << prev_offset <<std::endl;
        client = nh.serviceClient<apptronik_srvs::Float32>(full_set_service);
        srv = apptronik_srvs::Float32();

        srv.request.set_data = prev_offset - avg_val;

        if (client.call(srv))
        {
          ROS_INFO("Changed /%s/%s by %f", slave_names[i].c_str(), service_names[ii].c_str(), avg_val);
        }
        else
        {
          ROS_ERROR("Failed to call service: %s", full_set_service.c_str());
        }
      }
      else
      {
        ROS_ERROR("Failed to call service: %s", full_get_service.c_str());
      }

      sub_idx++;
    }
  }

  return 0;
}
