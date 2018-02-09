///////////////////////////////////////////////////////////////////////////////
//      Title     : demuxer.cpp
//      Project   : spacenav_demuxer
//      Created   : 2/8/2018
//      Author    : Veiko Vunder
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2017-2018. All rights reserved.
//
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#include "spacenav_demuxer/demuxer.h"

#include <string>
#include <exception>
#include <sstream>
#include <iostream>

Demuxer::Demuxer()
  : nh_("~"), last_button_state_(false)
{
  // subscribe to spacenav joy events
  std::string topic = "/spacenav/joy";
  nh_.getParam("spacenav_topic", topic);
  sub_ = nh_.subscribe(topic, 5, &Demuxer::joyCallback, this);
  pub_cam_ = nh_.advertise<sensor_msgs::Joy>("cam_joy", 100);
  pub_temoto_ = nh_.advertise<sensor_msgs::Joy>("temoto_joy", 100);

  // initialize in TEMOTO mode
  output_mode_ = OutputMode::TEMOTO;
  ROS_INFO("[Demuxer] Started in temoto control mode.");
}

Demuxer::~Demuxer()
{
  sub_.shutdown();
  pub_cam_.shutdown();
  pub_temoto_.shutdown();
}

void Demuxer::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{

  // Capture the rising edge of the left button signal
  // and change output mode if button was pressed
  if (msg->buttons.size())
  {
    if(msg->buttons[0] && !last_button_state_)
    {
      changeMode();
    }
    last_button_state_ = msg->buttons[0];
  }

  // Forward the incoming message to the selected publisher
  switch (output_mode_)
  {
    case OutputMode::VR_CAM:
      pub_cam_.publish(msg);
      break;

    case OutputMode::TEMOTO:
      pub_temoto_.publish(msg);
      break;

    default:
      ROS_ERROR("[Demuxer] Unhandled output mode.");
  }
}

void Demuxer::changeMode()
{
  switch (output_mode_)
  {
    case OutputMode::VR_CAM:
      output_mode_ = OutputMode::TEMOTO;
      ROS_INFO("[Demuxer] Switched to temoto control mode.");
      break;

    case OutputMode::TEMOTO:
      output_mode_ = OutputMode::VR_CAM;
      ROS_INFO("[Demuxer] Switched to camera control mode.");
      break;

    default:
      // change mode not defined for current output_mode
      ROS_WARN("[Demuxer] Unhandled output mode.");
      output_mode_ = OutputMode::TEMOTO;
  }
}

int main(int argc, char* argv[])
{
  // ROS init
  ros::init(argc, argv, "spacenav_demuxer");

  try
  {
    Demuxer demuxer;
    ros::spin();
  }
  catch(std::exception& e)
  {
    std::cout << "Got exception from Demuxer: " << e.what()<< std::endl;
  }
}
