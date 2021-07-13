#pragma once
#ifndef _FIRE_FINDER_H_
#define _FIRE_FINDER_H_

/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* long unsigned integer message */
#include <std_msgs/UInt64.h>

/* some STL includes */
#include <stdlib.h>
#include <stdio.h>
#include <mutex>

/* custom helper functions from our library */
#include <mrs_msgs/BacaProtocol.h>
#include <std_srvs/SetBool.h>


/* for calling simple ros services */
#include <std_srvs/Trigger.h>

//}

namespace ampoule_launcher
{

/* class Launcher //{ */
class Launcher : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

private:
  bool is_initialized_ = false;
  bool is_armed_ = false;

	// | ---------------- service server callbacks ---------------- |

  bool               callbackArmLauncher(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool               callbackFireLauncher(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_armLauncher_;
  ros::ServiceServer srv_server_fireLauncher_;

  ros::Publisher pub_serial_;

};
//}

}  // namespace fire_finder
#endif
