#ifndef GRIFFIN_DEMUXER_H
#define GRIFFIN_DEMUXER_H

#include <ros/ros.h>
#include "griffin_powermate/PowermateEvent.h"

class Demuxer
{
public:
  Demuxer(std::string rviz_ns, std::string griffin_ns);
  ~Demuxer();

private:

  void griffinCallback(const griffin_powermate::PowermateEvent& msg);

// nodehandles in rviz and griffin powermate namespaces
  ros::NodeHandle rviz_nh_;
  ros::NodeHandle griffin_nh_;

  // griffin event subscriber and muxed publishers
  ros::Subscriber sub_;
  ros::Publisher pub_cam_;
  ros::Publisher pub_temoto_;

  enum class OutputMode {VR_CAM, TEMOTO};
  OutputMode output_mode_;
};
#endif
