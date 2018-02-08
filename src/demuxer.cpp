#include "spacenav_demuxer/demuxer.h"

#include <sstream>
#include <string>
#include <vector>
#include <exception>

#include <iostream>

Demuxer::Demuxer(std::string rviz_ns, std::string griffin_ns)
  : griffin_nh_(griffin_ns), private_nh_("~")
{
  // subscribe to griffin events
  std::string topic = "events";
  griffin_nh_.getParam("griffin_topic", topic);
  sub_ = griffin_nh_.subscribe(topic, 5, &Demuxer::griffinCallback, this);
  pub_cam_ = private_nh_.advertise<griffin_powermate::PowermateEvent>("cam_event", 100);
  pub_temoto_ = private_nh_.advertise<griffin_powermate::PowermateEvent>("temoto_event", 100);
}

Demuxer::~Demuxer()
{
  sub_.shutdown();
}

void Demuxer::griffinCallback(const griffin_powermate::PowermateEvent& msg)
{
  if ()
  switch (output_mode_)
  {
    case OutputMode::VR_CAM:
      pub_cam_.publish(msg);

    case OutputMode::TEMOTO:
      pub_temoto_.publish(msg);

    default:
      ROS_ERROR("[Demuxer] Unhandled output mode.");
  }
}

int main(int argc, char* argv[])
{
  // ROS init
  ros::init(argc, argv, "griffin_demuxer");

  // get rviz and griffin namespaces
  std::string griffin_ns = "/griffin_powermate";
  ros::NodeHandle nh("~");
  nh.getParam("rviz_ns", rviz_ns);
  nh.getParam("griffin_ns", griffin_ns);

  try
  {
    Demuxer demuxer(griffin_ns);
    ros::spin();
  }
  catch(std::exception& e)
  {
    std::cout << "Got exception from Demuxer: " << e.what()<< std::endl;
  }
}
