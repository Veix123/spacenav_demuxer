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

#ifndef DEMUXER_H
#define DEMUXER_H

#include <ros/ros.h>
#include "sensor_msgs/Joy.h"

class Demuxer
{
public:
  Demuxer();
  ~Demuxer();

private:

  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void changeMode();

  ros::NodeHandle nh_;

  bool last_button_state_;

  // subscriber and muxed publishers of joy messages
  ros::Subscriber sub_;
  ros::Publisher pub_cam_;
  ros::Publisher pub_temoto_;

  enum class OutputMode {VR_CAM, TEMOTO};
  OutputMode output_mode_;
};
#endif
