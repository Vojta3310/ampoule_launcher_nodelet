#include "launcher.h"
/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>

namespace ampoule_launcher
{

/* onInit() method //{ */
void Launcher::onInit() {

  /* obtain node handle */
  ros::NodeHandle nh("~");

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------ initialize publishers ----------------- |
  pub_serial_ = nh.advertise<mrs_msgs::BacaProtocol>("serial_out", 0);

  // | --------------- initialize service servers --------------- |
  srv_server_armLauncher_  = nh.advertiseService("arm_in", &Launcher::callbackArmLauncher, this);
  srv_server_fireLauncher_ = nh.advertiseService("launch_in", &Launcher::callbackFireLauncher, this);


  ROS_INFO_ONCE("[Launcher]: initialized");


  is_initialized_ = true;
}
//}


// | -------------------- service callbacks ------------------- |

/* //{ callbackFireLauncher() */

bool Launcher::callbackFireLauncher([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (!is_armed_) {
    ROS_ERROR("[Launcher]: Can not fire launcher. Launcher is not armed!");
    res.success = false;
    return false;
  }

  ROS_ERROR("[Launcher]: Firing Launcher !!");
  mrs_msgs::BacaProtocol msg;
  msg.stamp = ros::Time::now();
  msg.payload.push_back(0x39);
  try {
    pub_serial_.publish(msg);
    res.success = true;
    is_armed_   = false;
    return true;
  }
  catch (...) {
    res.success = false;
  }
  return false;
}
//}

/* //{ callbackArmLauncher() */

bool Launcher::callbackArmLauncher([[maybe_unused]] std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
  mrs_msgs::BacaProtocol msg;
  msg.stamp = ros::Time::now();
  if (req.data) {
    ROS_INFO("[Launcher]: Arm Launcher !!");
    msg.payload.push_back(0x38);
  } else {
    ROS_INFO("[Launcher]: Disarm Launcher !!");
    msg.payload.push_back(0x37);
  }
  try {
    pub_serial_.publish(msg);
    is_armed_   = req.data;
    res.success = true;
    return true;
  }
  catch (...) {
    res.success = false;
  }
  return false;
}
//}


}  // namespace ampoule_launcher

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(ampoule_launcher::Launcher, nodelet::Nodelet);
