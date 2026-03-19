/**
 * run_pub_f_with_wind.cpp — 风速话题桥
 *
 * 订阅 ROS 风速话题，发布到 gz /wind_vector。AerodynamicsPlugin 用 真实空速 = 地速 - 风速 算气动力。
 * 与 run_pub_f 同时运行时，插件用 v_from_ros_odom（来自 /ekf/ekf_odom），地速为 local NED，
 * 故风速也须为 NED：x=北向(m/s)，y=东向，z=地向(下为正)。
 *
 * ROS 话题（NED，与 /ekf/ekf_odom 的 twist 同系）：
 *   1) /wind_cmd_vector (geometry_msgs/Vector3): (x, y, z) = (北, 东, 下) m/s
 *   2) /wind_speed_direction (Float64MultiArray): [speed_m_s, yaw_rad]
 *      水平风：yaw=0 为北向(+x)，yaw=pi/2 为东向(+y)，符合 NED
 */

#include <cmath>
#include <string>

#include <gz/transport/Node.hh>
#include <gz/msgs.hh>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3.h>

gz::transport::Node node;
gz::transport::Node::Publisher wind_vector_pub;

double wind_x = 0.0, wind_y = 0.0, wind_z = 0.0;

void windVectorCb(const geometry_msgs::Vector3ConstPtr& msg) {
  wind_x = msg->x;
  wind_y = msg->y;
  wind_z = msg->z;
  ROS_INFO_THROTTLE(2.0, "[wind] vector wx=%.2f wy=%.2f wz=%.2f m/s", wind_x, wind_y, wind_z);
}

void windSpeedDirectionCb(const std_msgs::Float64MultiArrayConstPtr& msg) {
  if (msg->data.size() < 2) return;
  double speed = msg->data[0];
  double yaw   = msg->data[1];
  wind_x = speed * std::cos(yaw);
  wind_y = speed * std::sin(yaw);
  wind_z = 0.0;
  ROS_INFO_THROTTLE(2.0, "[wind] speed=%.2f m/s yaw=%.1f deg", speed, yaw * 180.0 / M_PI);
}

void timerCb(const ros::TimerEvent&) {
  gz::msgs::Vector3d msg;
  msg.set_x(wind_x);
  msg.set_y(wind_y);
  msg.set_z(wind_z);
  wind_vector_pub.Publish(msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "wind_cmd_bridge");
  ros::NodeHandle nh;

  wind_vector_pub = node.Advertise<gz::msgs::Vector3d>("/wind_vector");
  ros::Subscriber sub_vec  = nh.subscribe("/wind_cmd_vector", 1, &windVectorCb);
  ros::Subscriber sub_sd   = nh.subscribe("/wind_speed_direction", 1, &windSpeedDirectionCb);

  ros::Timer timer = nh.createTimer(ros::Duration(0.02), timerCb);  // 50 Hz 发到 gz
  ros::spin();
  return 0;
}
