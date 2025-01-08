
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
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <string>
#include <visualization_msgs/MarkerArray.h>
using namespace std;
gz::transport::Node node;
gz::transport::Node::Publisher pub,pub2;
ros::Subscriber targetSub,time_sub,howbig_sub,endSub,odom_sub;
ros::Publisher  marker_pub,delay_odom_pub;
double continue_time=0;
double howbig_wind=0;
float radius = 5; // 圆的半径

nav_msgs::Odometry::ConstPtr now_pose;

// std::string filename = "/home/cybird/文档/new_px4/wind_sim/wind2.csv";
// std::ofstream outFile("/home/cybird/文档/new_px4/wind_sim/wind.csv");
std::string filename = ("/home/cybird/下载/px4-n/px4-v1.14.0-stable/wind_sim/wind.csv");
std::ofstream outFile("/home/cybird/下载/px4-n/px4-v1.14.0-stable/wind_sim/wind_sim_out.csv");

int count_wind_area=0;
struct windaera{
  double center_x=0;
  double center_y=0;
  double center_z=0;

  double wind_x=0;
  double wind_y=0;
  double orientation_x=0;
  double orientation_y=0;
  double orientation_z=0;
  double orientation_w=1;
};
bool have_odom=false;
std::vector<windaera> Areas;
bool if_have_prepared_file=true;
vector<nav_msgs::Odometry> odom_buffer;
ros::Time get_cmdtime;

nav_msgs::Odometry delay_odom;
bool have_delay_odom=false;
void goal_Cb(const geometry_msgs::PoseStamped::ConstPtr& msg){    
    if (if_have_prepared_file) {
        return;
    }
    else{
      std::cerr << "dont have a prepared file ,need new 3 area to record" << std::endl;

      count_wind_area++;
            // Extract yaw angle
      if(count_wind_area<=3){
        double yaw = tf::getYaw(msg->pose.orientation);
        double wind_x = cos(yaw);
        double wind_y = sin(yaw);

        windaera Area;
        Area.center_x= msg->pose.position.x;
        Area.center_y= msg->pose.position.y;
        Area.center_z= 1.1;
        Area.wind_x=wind_x;
        Area.wind_y=wind_y;

        Area.orientation_x=msg->pose.orientation.x;
        Area.orientation_y=msg->pose.orientation.y;
        Area.orientation_z=msg->pose.orientation.z;
        Area.orientation_w=msg->pose.orientation.w;

        Areas.push_back(Area);


        outFile << std::fixed << std::setprecision(6)
                    << Area.center_x << ", " <<  Area.center_y << ", " << Area.center_z << ", "
                    << wind_x << ", " << wind_y<<", " << Area.orientation_x<<", " <<Area.orientation_y<<", " << Area.orientation_z<<", " << Area.orientation_w<< "\n";


        if(count_wind_area==3)
            outFile.close();
        
        }
    }

}
void initialpose_Cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    gz::msgs::Vector3d flag;
    pub2.Publish(flag);
}

void time_Cb(const std_msgs::Float64::ConstPtr& msg){

continue_time=msg->data;
std::cout<<"continue_time"<<continue_time<<std::endl;

}
void howbig_Cb(const std_msgs::Float64::ConstPtr& msg){

  howbig_wind=msg->data;
  std::cout<<"howbig_wind"<<howbig_wind<<std::endl;

}
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
  now_pose=msg;
  if (have_odom==false)
	{

    have_odom=true;
		odom_buffer.emplace_back(*now_pose);
		get_cmdtime = ros::Time::now();

	}
	else
	{
		odom_buffer.emplace_back(*now_pose);

		if ((ros::Time::now() - get_cmdtime).toSec() > 0.01)//延迟0.05s
		{
			delay_odom = odom_buffer[0];
      // std::cout<<"delay_odom"<<delay_odom.twist.twist.linear.x<<std::endl;
			odom_buffer.erase(odom_buffer.begin());
      have_delay_odom=true;
      delay_odom_pub.publish(delay_odom);
		}
	}
}
void pub_delay_odom(const ros::TimerEvent& event) {
    if (have_delay_odom) {
        delay_odom_pub.publish(delay_odom);
    }
}

void timerCallback(const ros::TimerEvent& event) {
    if(! have_odom){
      return;
    }
    int now_in_which_area=-999;
    int seq_area=0;
    for (int i=0;i<Areas.size();i++) {
            windaera Area=Areas[i];
            seq_area++;
            visualization_msgs::MarkerArray marker_array;
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "circle";
            marker.id =seq_area;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            // std::cout<<"Area"<<Area.center_x<<Area.center_y<<Area.center_z<<std::endl;
            marker.pose.position.x = Area.center_x;
            marker.pose.position.y = Area.center_y;
            marker.pose.position.z = Area.center_z;
            marker.pose.orientation.x = Area.orientation_x;
            marker.pose.orientation.y = Area.orientation_y;
            marker.pose.orientation.z = Area.orientation_z;
            marker.pose.orientation.w = Area.orientation_w;


            marker.scale.x = radius * 2;
            marker.scale.y = radius * 2;
            marker.scale.z = radius * 2;
            marker.color.a = 0.2; 
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            marker_array.markers.push_back(marker);

        // 创建箭头
            visualization_msgs::Marker marker_arrow;
            marker_arrow.header.frame_id = "map";
            marker_arrow.header.stamp = ros::Time::now();
            marker_arrow.ns = "arrow";
            marker_arrow.id = seq_area*10;
            marker_arrow.type = visualization_msgs::Marker::ARROW;
            marker_arrow.action = visualization_msgs::Marker::ADD;
            marker_arrow.pose.position.x = Area.center_x;
            marker_arrow.pose.position.y = Area.center_y;
            marker_arrow.pose.position.z = Area.center_z;
            marker_arrow.pose.orientation.x = Area.orientation_x;
            marker_arrow.pose.orientation.y = Area.orientation_y;
            marker_arrow.pose.orientation.z = Area.orientation_z;
            marker_arrow.pose.orientation.w = Area.orientation_w;
            marker_arrow.scale.x = 5.0; // 箭头的长度
            marker_arrow.scale.y = 0.1; // 箭头的宽度
            marker_arrow.scale.z = 0.1;
            marker_arrow.color.a = 1.0;
            marker_arrow.color.r = 0.0;
            marker_arrow.color.g = 1.0;
            marker_arrow.color.b = 0.0;
            marker_array.markers.push_back(marker_arrow);



            marker_pub.publish(marker_array);
            float x = now_pose->pose.pose.position.x;
            float y = now_pose->pose.pose.position.y;
            float z = now_pose->pose.pose.position.z;

            float distance_to_center = sqrt(pow(x -Area.center_x, 2) + pow(y - Area.center_y, 2) + pow(z - Area.center_z, 2));
            if (distance_to_center <= radius){
              now_in_which_area=seq_area;
              std::cout<<"now_in_which_area"<<now_in_which_area<<std::endl;
            }
    }

    if (now_in_which_area>0){//1 2 3
        gz::msgs::Vector3d msg_wind;
        double now_windx=Areas[now_in_which_area-1].wind_x;
        double now_windy=Areas[now_in_which_area-1].wind_y;
        msg_wind.set_x(now_windx);
        msg_wind.set_y(now_windy);
        msg_wind.set_z(howbig_wind); // Assuming z-component is not used
        pub.Publish(msg_wind);
    }
    else{//-999
        gz::msgs::Vector3d flag;
        pub2.Publish(flag);
    }

    // for(int i=0;i<100*continue_time;i++){
    // gz::msgs::Vector3d msg_wind;
    //     msg_wind.set_x(wind_x);
    //     msg_wind.set_y(wind_y);
    //     msg_wind.set_z(howbig_wind); // Assuming z-component is not used
    // pub.Publish(msg_wind);
    // ros::Duration(0.01).sleep();
    // }
    // gz::msgs::Vector3d flag;
    // pub2.Publish(flag);

}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "get_windreal");
  ros::NodeHandle nh;


  pub = node.Advertise<gz::msgs::Vector3d>("/direction");
  pub2 = node.Advertise<gz::msgs::Vector3d>("/end_wind");
  targetSub = nh.subscribe("/move_base_simple/goal", 1, &goal_Cb,
                                 ros::TransportHints().tcpNoDelay()); 
  endSub = nh.subscribe("/initialpose", 1, &initialpose_Cb,
                                 ros::TransportHints().tcpNoDelay()); 
                                 
  time_sub= nh.subscribe("/how_long", 1, &time_Cb,
                                 ros::TransportHints().tcpNoDelay()); 
  howbig_sub= nh.subscribe("/howbig", 1, &howbig_Cb,
                                 ros::TransportHints().tcpNoDelay()); 

  odom_sub= nh.subscribe("/mavros/local_position/odom", 1, &odom_cb,
                                 ros::TransportHints().tcpNoDelay()); 
  delay_odom_pub = nh.advertise<nav_msgs::Odometry>("/delay_odom", 1);

  ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);
  ros::Timer timer2 = nh.createTimer(ros::Duration(0.001), pub_delay_odom);

  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/wind_Area", 10);//写一个 接受2dgoal 接受一个发布一个对应位置的球   写到yaml 读取 下次用

std::cout<< "in main" << std::endl;
std::cerr << "2" << std::endl;
        std::ifstream file(filename);
        if (file.is_open()) {
            std::cerr << "already have a prepared file " << std::endl;
            std::string line;
                    int line_index=0;
                    while (std::getline(file, line)) {
                        line_index++;
                            std::istringstream ss(line);
                            double cellValue;
                            std::string delimiter;
                            int index=0;
                            windaera Area;
                            // 以逗号为分隔符提取每个数字
                            while (std::getline(ss, delimiter, ',')) {
                                // 跳过前六个数字
                                if (index == 0) {
                                    index++;
                                    Area.center_x=std::stod(delimiter);
                                    continue;
                                }
                                if(index == 1){
                                    index++;
                                    Area.center_y=std::stod(delimiter);
                                    continue;
                                }
                                if(index == 2){
                                    index++;
                                    Area.center_z=std::stod(delimiter);
                                    continue;
                                }
                                if (index == 3) {
                                    index++;
                                    Area.wind_x=std::stod(delimiter);
                                    continue;
                                }
                                if(index == 4){
                                    index++;
                                    Area.wind_y=std::stod(delimiter);
                                    continue;
                                }
                                if(index == 5){
                                    index++;
                                    Area.orientation_x=std::stod(delimiter);
                                    continue;
                                }            
                                if(index == 6){
                                    index++;
                                    Area.orientation_y=std::stod(delimiter);
                                    continue;
                                }    
                                if(index == 7){
                                    index++;
                                    Area.orientation_z=std::stod(delimiter);

                                    continue;
                                }    
                                if(index == 8){
                                    index++;
                                    Area.orientation_w=std::stod(delimiter);
                                    Areas.push_back(Area);    
                                    std::cout<<"   Areas.size()"<<Areas.size()<<std::endl;
                                    break;
                                }                                  

                            }

                    }
                    if_have_prepared_file=true;
                    file.close();
    
        }
        else{
          std::cerr << "---!!!dont have a prepared file " << std::endl;
          if_have_prepared_file=false;
        }

  ros::spin();
}