
#include <algorithm>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include <gz/math/Quaternion.hh>
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/Pose.hh"
// #include "gz/msgs/wrench.pb.h"
#include "gz/msgs.hh"
#include "gz/transport.hh"
#include <memory>
#include <gz/sim/System.hh>
#include <ros/ros.h>
#include "gz/transport.hh"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
#include <string>
#include <visualization_msgs/MarkerArray.h>
using namespace std;
gz::transport::Node node;
ros::Publisher  pub_fxyz,pub_torque,pub_force;
ros::Subscriber imu_sub,odom_sub;
gz::transport::Node::Publisher q_from_ros_imu_pub,v_from_ros_odom_pub;
double fx=0;
double fy=0;
double fz=0;

double px=0;
double py=0;
double pz=0;

double alpha=0;
double beta_=0;
double Vx=0;
double Vy=0;
double Vz=0;
double w=0;
double x=0;
double y=0;
double z=0;

double w2=0;
double x2=0;
double y2=0;
double z2=0;

double Vx1=0;
double Vy1=0;
double Vz1=0;

double torquex=0;
double torquey=0;
double torquez=0;

void timerCallback(const ros::TimerEvent& event) {
std_msgs::Float64MultiArray fair;
 fair.data.push_back(fx);
 fair.data.push_back(fy);
 fair.data.push_back(fz);
 fair.data.push_back(alpha);
 fair.data.push_back(beta_);
 fair.data.push_back(Vx);
 fair.data.push_back(Vy);
 fair.data.push_back(Vz);
 fair.data.push_back(w);
 fair.data.push_back(x);
 fair.data.push_back(y);
 fair.data.push_back(z);
 fair.data.push_back(Vx1);
 fair.data.push_back(Vy1);
 fair.data.push_back(Vz1);
 fair.data.push_back(w2);
 fair.data.push_back(x2);
 fair.data.push_back(y2);
 fair.data.push_back(z2);

 fair.data.push_back(px);
 fair.data.push_back(py);
 fair.data.push_back(pz);
pub_fxyz.publish(fair);


std_msgs::Float64MultiArray force;
 force.data.push_back(fx);
 force.data.push_back(fy);
 force.data.push_back(fz);
pub_force.publish(force);


std_msgs::Float64MultiArray torque;
 torque.data.push_back(torquex);
 torque.data.push_back(torquey);
 torque.data.push_back(torquez);

pub_torque.publish(torque);
}
void OnReceiveMessage(const gz::msgs::Vector3d& msg)
{
    // 使用成员函数来获取向量的各个分量
    fx = msg.x();
    fy = msg.y();
    fz = msg.z();


}
void OnReceiveMessage2(const gz::msgs::Vector2d& msg)
{
    // 使用成员函数来获取向量的各个分量
    alpha = msg.x();
    beta_ = msg.y();
}
void OnReceiveMessage3(const gz::msgs::Vector3d& msg)
{
    // 使用成员函数来获取向量的各个分量
    Vx = msg.x();
    Vy = msg.y();
    Vz = msg.z();
}
void OnReceiveMessage4(const gz::msgs::Quaternion& msg)
{
  gz::msgs::Quaternion q_msg=msg;
    w = q_msg.w();
    x = q_msg.x();
    y = q_msg.y();
    z = q_msg.z();

}
void OnReceiveMessage6(const gz::msgs::Quaternion& msg)
{
  gz::msgs::Quaternion q_msg=msg;
    w2 = q_msg.w();
    x2 = q_msg.x();
    y2 = q_msg.y();
    z2 = q_msg.z();

}
void OnReceiveMessage_moment(const gz::msgs::Vector3d& msg)
{
    // 使用成员函数来获取向量的各个分量
    torquex = msg.x();
    torquey = msg.y();
    torquez = msg.z();

}


void OnReceiveMessage5(const gz::msgs::Vector3d& msg)
{
    // 使用成员函数来获取向量的各个分量
    Vx1 = msg.x();
    Vy1 = msg.y();
    Vz1 = msg.z();
}
void OnReceiveMessage7(const gz::msgs::Vector3d& msg)
{
    // 使用成员函数来获取向量的各个分量
    px = msg.x();
    py = msg.y();
    pz = msg.z();


}
void imuCallBack(sensor_msgs::ImuConstPtr pMsg)
    {
        sensor_msgs::Imu msg;
        msg = *pMsg;
        double w1 = msg.orientation.w;
        double x1 = msg.orientation.x;
        double y1 = msg.orientation.y;
        double z1 = msg.orientation.z;
  gz::math::Quaternion q1(-w1,-x1,-y1,-z1);
  gz::math::Quaternion q2(0.0,0.707,0.0,0.707);
  gz::math::Quaternion q_=q1*q2;

        gz::msgs::Quaternion q_msg;
        q_msg.set_w(q_.W());
        q_msg.set_x(q_.X());
        q_msg.set_y(q_.Y());
        q_msg.set_z(q_.Z());
        q_from_ros_imu_pub.Publish(q_msg);

    }

void odomCallBack(nav_msgs::OdometryConstPtr pMsg)
{
    auto  msg = *pMsg;
    double vx = msg.twist.twist.linear.x;
    double vy = msg.twist.twist.linear.y;
    double vz = msg.twist.twist.linear.z;

        gz::msgs::Vector3d v_msg;
  v_msg.set_x(vx);
  v_msg.set_y(vy);
  v_msg.set_z(vz);
        v_from_ros_odom_pub.Publish(v_msg);
}
int main(int argc, char **argv) 
{
  ros::init(argc, argv, "get_wind");
  ros::NodeHandle nh;

  node.Subscribe("/tailsitter_0/zhang_force", &OnReceiveMessage);
  node.Subscribe("/tailsitter_0/alpha_beta", &OnReceiveMessage2);
  node.Subscribe("/tailsitter_0/plugin_v", &OnReceiveMessage3);

  node.Subscribe("/tailsitter_0/plugin_q", &OnReceiveMessage4);
  node.Subscribe("/tailsitter_0/plugin_p", &OnReceiveMessage7);
  node.Subscribe("/tailsitter_0/plugin_v2", &OnReceiveMessage5);
  node.Subscribe("/tailsitter_0/plugin_q2", &OnReceiveMessage6);
  pub_fxyz = nh.advertise<std_msgs::Float64MultiArray>("/fair", 1);
  pub_force= nh.advertise<std_msgs::Float64MultiArray>("/force_gz", 1);
  pub_torque = nh.advertise<std_msgs::Float64MultiArray>("/torque_gz", 1);
  node.Subscribe("/tailsitter_0/moment", &OnReceiveMessage_moment);
  
  imu_sub = nh.subscribe("/mavros/imu/data", 1, &imuCallBack);
  odom_sub = nh.subscribe("/ekf/ekf_odom", 1, &odomCallBack);
  q_from_ros_imu_pub= node.Advertise<gz::msgs::Quaternion>("q_from_ros_imu");
  v_from_ros_odom_pub= node.Advertise<gz::msgs::Vector3d>("v_from_ros_odom");
  ros::Timer timer = nh.createTimer(ros::Duration(0.01), timerCallback);

    

  ros::spin();
}